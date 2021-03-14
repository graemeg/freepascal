{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020,2021 by the Free Pascal development team.

    API functions for The WebAssembly System Interface (WASI).

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function __wasi_args_get(
  argv: PPUInt8;
  argv_buf: PUInt8
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'args_get';

function __wasi_args_sizes_get(
  argc: P__wasi_size_t;
  argv_buf_size: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'args_sizes_get';

function __wasi_environ_get(
  environ: PPUInt8;
  environ_buf: PUInt8
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'environ_get';

function __wasi_environ_sizes_get(
  environc: P__wasi_size_t;
  environ_buf_size: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'environ_sizes_get';

function __wasi_clock_res_get(
  id: __wasi_clockid_t;
  resolution: P__wasi_timestamp_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'clock_res_get';

function __wasi_clock_time_get(
  id: __wasi_clockid_t;
  precision: __wasi_timestamp_t;
  time: P__wasi_timestamp_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'clock_time_get';

function __wasi_fd_advise(
  fd: __wasi_fd_t;
  offset: __wasi_filesize_t;
  len: __wasi_filesize_t;
  advice: __wasi_advice_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_advise';

function __wasi_fd_allocate(
  fd: __wasi_fd_t;
  offset: __wasi_filesize_t;
  len: __wasi_filesize_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_allocate';

function __wasi_fd_close(
  fd: __wasi_fd_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_close';

function __wasi_fd_datasync(
  fd: __wasi_fd_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_datasync';

function __wasi_fd_fdstat_get(
  fd: __wasi_fd_t;
  stat: P__wasi_fdstat_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_fdstat_get';

function __wasi_fd_fdstat_set_flags(
  fd: __wasi_fd_t;
  flags: __wasi_fdflags_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_fdstat_set_flags';

function __wasi_fd_fdstat_set_rights(
  fd: __wasi_fd_t;
  fs_rights_base: __wasi_rights_t;
  fs_rights_inheriting: __wasi_rights_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_fdstat_set_rights';

function __wasi_fd_filestat_get(
  fd: __wasi_fd_t;
  buf: P__wasi_filestat_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_filestat_get';

function __wasi_fd_filestat_set_size(
  fd: __wasi_fd_t;
  size: __wasi_filesize_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_filestat_set_size';

function __wasi_fd_filestat_set_times(
  fd: __wasi_fd_t;
  atim: __wasi_timestamp_t;
  mtim: __wasi_timestamp_t;
  fst_flags: __wasi_fstflags_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_filestat_set_times';

function __wasi_fd_pread(
  fd: __wasi_fd_t;
  iovs: {const} P__wasi_iovec_t;
  iovs_len: size_t;
  offset: __wasi_filesize_t;
  nread: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_pread';

function __wasi_fd_prestat_get(
  fd: __wasi_fd_t;
  buf: P__wasi_prestat_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_prestat_get';

function __wasi_fd_prestat_dir_name(
  fd: __wasi_fd_t;
  path: PUInt8;
  path_len: __wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_prestat_dir_name';

function __wasi_fd_pwrite(
  fd: __wasi_fd_t;
  iovs: {const} P__wasi_ciovec_t;
  iovs_len: size_t;
  offset: __wasi_filesize_t;
  nwritten: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_pwrite';

function __wasi_fd_read(
  fd: __wasi_fd_t;
  iovs: {const} P__wasi_iovec_t;
  iovs_len: size_t;
  nread: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_read';

function __wasi_fd_readdir(
  fd: __wasi_fd_t;
  buf: PUInt8;
  buf_len: __wasi_size_t;
  cookie: __wasi_dircookie_t;
  bufused: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_readdir';

function __wasi_fd_renumber(
  fd: __wasi_fd_t;
  to_: __wasi_fd_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_renumber';

function __wasi_fd_seek(
  fd: __wasi_fd_t;
  offset: __wasi_filedelta_t;
  whence: __wasi_whence_t;
  newoffset: P__wasi_filesize_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_seek';

function __wasi_fd_sync(
  fd: __wasi_fd_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_sync';

function __wasi_fd_tell(
  fd: __wasi_fd_t;
  offset: P__wasi_filesize_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_tell';

function __wasi_fd_write(
  fd: __wasi_fd_t;
  iovs: {const} P__wasi_ciovec_t;
  iovs_len: size_t;
  nwritten: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'fd_write';

function __wasi_path_create_directory(
  fd: __wasi_fd_t;
  const path: PChar;
  path_len: size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_create_directory';

function __wasi_path_filestat_get(
  fd: __wasi_fd_t;
  flags: __wasi_lookupflags_t;
  const path: PChar;
  path_len: size_t;
  buf: P__wasi_filestat_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_filestat_get';

function __wasi_path_filestat_set_times(
  fd: __wasi_fd_t;
  flags: __wasi_lookupflags_t;
  const path: PChar;
  path_len: size_t;
  atim: __wasi_timestamp_t;
  mtim: __wasi_timestamp_t;
  fst_flags: __wasi_fstflags_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_filestat_set_times';

function __wasi_path_link(
  old_fd: __wasi_fd_t;
  old_flags: __wasi_lookupflags_t;
  const old_path: PChar;
  old_path_len: size_t;
  new_fd: __wasi_fd_t;
  const new_path: PChar;
  new_path_len: size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_link';

function __wasi_path_open(
  fd: __wasi_fd_t;
  dirflags: __wasi_lookupflags_t;
  const path: PChar;
  path_len: size_t;
  oflags: __wasi_oflags_t;
  fs_rights_base: __wasi_rights_t;
  fs_rights_inherting: __wasi_rights_t;
  fdflags: __wasi_fdflags_t;
  opened_fd: P__wasi_fd_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_open';

function __wasi_path_readlink(
  fd: __wasi_fd_t;
  const path: PChar;
  path_len: size_t;
  buf: PUInt8;
  buf_len: __wasi_size_t;
  bufused: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_readlink';

function __wasi_path_remove_directory(
  fd: __wasi_fd_t;
  const path: PChar;
  path_len: size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_remove_directory';

function __wasi_path_rename(
  fd: __wasi_fd_t;
  const old_path: PChar;
  old_path_len: size_t;
  new_fd: __wasi_fd_t;
  const new_path: PChar;
  new_path_len: size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_rename';

function __wasi_path_symlink(
  const old_path: PChar;
  old_path_len: size_t;
  fd: __wasi_fd_t;
  const new_path: PChar;
  new_path_len: size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_symlink';

function __wasi_path_unlink_file(
  fd: __wasi_fd_t;
  const path: PChar;
  path_len: size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'path_unlink_file';

function __wasi_poll_oneoff(
  in_ : {const} P__wasi_subscription_t;
  out_: P__wasi_event_t;
  nsubscriptions: __wasi_size_t;
  nevents: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'poll_oneoff';

procedure __wasi_proc_exit(
  rval: __wasi_exitcode_t
); noreturn; external 'wasi_snapshot_preview1' name 'proc_exit';

function  __wasi_proc_raise(
  sig: __wasi_signal_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'proc_raise';

function __wasi_sched_yield(
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'sched_yield';

function __wasi_random_get(
  buf: PUInt8;
  buf_len: __wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'random_get';

function __wasi_sock_recv(
  fd: __wasi_fd_t;
  const ri_data: P__wasi_iovec_t;
  ri_data_len: size_t;
  ri_flags: __wasi_riflags_t;
  ro_datalen: P__wasi_size_t;
  ro_flags: P__wasi_roflags_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'sock_recv';

function __wasi_sock_send(
  fd: __wasi_fd_t;
  si_data: {const} P__wasi_ciovec_t;
  si_data_len: size_t;
  si_flags: __wasi_siflags_t;
  so_datalen: P__wasi_size_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'sock_send';

function __wasi_sock_shutdown(
  fd: __wasi_fd_t;
  how: __wasi_sdflags_t
): __wasi_errno_t; external 'wasi_snapshot_preview1' name 'sock_shutdown';

