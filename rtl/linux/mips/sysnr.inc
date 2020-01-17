{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003-2004 by Florian Klaempfl and David Zhang

    Syscall nrs for mips-linux O32

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{
* This file contains the system call numbers.
}

Const
  __nr_Linux                            =  4000;
  syscall_nr_exit                       =  __nr_linux +  1;
  syscall_nr_fork                       =  __nr_linux +  2;
  syscall_nr_read                       =  __nr_linux +  3;
  syscall_nr_write                      =  __nr_linux +  4;
  syscall_nr_open                       =  __nr_linux +  5;
  syscall_nr_close                      =  __nr_linux +  6;
  syscall_nr_waitpid                    =  __nr_linux +  7;
  syscall_nr_creat                      =  __nr_linux +  8;
  syscall_nr_link                       =  __nr_linux +  9;
  syscall_nr_unlink                     =  __nr_linux + 10;
  syscall_nr_execve                     =  __nr_linux + 11;
  syscall_nr_chdir                      =  __nr_linux + 12;
  syscall_nr_time                       =  __nr_linux + 13;
  syscall_nr_mknod                      =  __nr_linux + 14;
  syscall_nr_chmod                      =  __nr_linux + 15;
  syscall_nr_lchown                     =  __nr_linux + 16;
  syscall_nr_break                      =  __nr_linux + 17;

  syscall_nr_unused18                   = __nr_Linux +  18;

  syscall_nr_lseek                      =  __nr_linux + 19;
  syscall_nr_getpid                     =  __nr_linux + 20;
  syscall_nr_mount                      =  __nr_linux + 21;
  syscall_nr_umount                     =  __nr_linux + 22;
  syscall_nr_setuid                     =  __nr_linux + 23;
  syscall_nr_getuid                     =  __nr_linux + 24;
  syscall_nr_stime                      =  __nr_linux + 25;
  syscall_nr_ptrace                     =  __nr_linux + 26;
  syscall_nr_alarm                      =  __nr_linux + 27;

  syscall_nr_unused28                   = __nr_Linux +  28;

  syscall_nr_pause                      =  __nr_linux + 29;
  syscall_nr_utime                      =  __nr_linux + 30;
  syscall_nr_stty                       =  __nr_linux + 31;
  syscall_nr_gtty                       =  __nr_linux + 32;
  syscall_nr_access                     =  __nr_linux + 33;
  syscall_nr_nice                       =  __nr_linux + 34;
  syscall_nr_ftime                      =  __nr_linux + 35;
  syscall_nr_sync                       =  __nr_linux + 36;
  syscall_nr_kill                       =  __nr_linux + 37;
  syscall_nr_rename                     =  __nr_linux + 38;
  syscall_nr_mkdir                      =  __nr_linux + 39;
  syscall_nr_rmdir                      =  __nr_linux + 40;
  syscall_nr_dup                        =  __nr_linux + 41;
  syscall_nr_pipe                       =  __nr_linux + 42;
  syscall_nr_times                      =  __nr_linux + 43;
  syscall_nr_prof                       =  __nr_linux + 44;
  syscall_nr_brk                        =  __nr_linux + 45;
  syscall_nr_setgid                     =  __nr_linux + 46;
  syscall_nr_getgid                     =  __nr_linux + 47;
  syscall_nr_signal                     =  __nr_linux + 48;
  syscall_nr_geteuid                    =  __nr_linux + 49;
  syscall_nr_getegid                    =  __nr_linux + 50;
  syscall_nr_acct                       =  __nr_linux + 51;
  syscall_nr_umount2                    =  __nr_linux + 52;
  syscall_nr_lock                       =  __nr_linux + 53;
  syscall_nr_ioctl                      =  __nr_linux + 54;
  syscall_nr_fcntl                      =  __nr_linux + 55;
  syscall_nr_mpx                        =  __nr_linux + 56;
  syscall_nr_setpgid                    =  __nr_linux + 57;
  syscall_nr_ulimit                     =  __nr_linux + 58;

  syscall_nr_unused59                   = __nr_Linux +  59;

  syscall_nr_umask                      =  __nr_linux + 60;
  syscall_nr_chroot                     =  __nr_linux + 61;
  syscall_nr_ustat                      =  __nr_linux + 62;
  syscall_nr_dup2                       =  __nr_linux + 63;
  syscall_nr_getppid                    =  __nr_linux + 64;
  syscall_nr_getpgrp                    =  __nr_linux + 65;
  syscall_nr_setsid                     =  __nr_linux + 66;
  syscall_nr_sigaction                  =  __nr_linux + 67;
  syscall_nr_sgetmask                   =  __nr_linux + 68;
  syscall_nr_ssetmask                   =  __nr_linux + 69;
  syscall_nr_setreuid                   =  __nr_linux + 70;
  syscall_nr_setregid                   =  __nr_linux + 71;
  syscall_nr_sigsuspend                 =  __nr_linux + 72;
  syscall_nr_sigpending                 =  __nr_linux + 73;
  syscall_nr_sethostname                =  __nr_linux + 74;
  syscall_nr_setrlimit                  =  __nr_linux + 75;
  syscall_nr_getrlimit                  =  __nr_linux + 76;
  syscall_nr_getrusage                  =  __nr_linux + 77;
  syscall_nr_gettimeofday               =  __nr_linux + 78;
  syscall_nr_settimeofday               =  __nr_linux + 79;
  syscall_nr_getgroups                  =  __nr_linux + 80;
  syscall_nr_setgroups                  =  __nr_linux + 81;

//  syscall_nr_select                     =  __nr_linux + 82;
  syscall_nr_reserved82                 =  __nr_Linux +  82;

  syscall_nr_symlink                    =  __nr_linux + 83;

  syscall_nr_unused84                   = __nr_Linux +  84;

  syscall_nr_readlink                   =  __nr_linux + 85;
  syscall_nr_uselib                     =  __nr_linux + 86;
  syscall_nr_swapon                     =  __nr_linux + 87;
  syscall_nr_reboot                     =  __nr_linux + 88;
  syscall_nr_readdir                    =  __nr_linux + 89;
  syscall_nr_mmap                       =  __nr_linux + 90;
  syscall_nr_munmap                     =  __nr_linux + 91;
  syscall_nr_truncate                   =  __nr_linux + 92;
  syscall_nr_ftruncate                  =  __nr_linux + 93;
  syscall_nr_fchmod                     =  __nr_linux + 94;
  syscall_nr_fchown                     =  __nr_linux + 95;
  syscall_nr_getpriority                =  __nr_linux + 96;
  syscall_nr_setpriority                =  __nr_linux + 97;
  syscall_nr_profil                     =  __nr_linux + 98;
  syscall_nr_statfs                     =  __nr_linux + 99;
  syscall_nr_fstatfs                    = __nr_linux + 100;
  syscall_nr_ioperm                     = __nr_linux + 101;
  syscall_nr_socketcall                 = __nr_linux + 102;
  syscall_nr_syslog                     = __nr_linux + 103;
  syscall_nr_setitimer                  = __nr_linux + 104;
  syscall_nr_getitimer                  = __nr_linux + 105;
  syscall_nr_stat                       = __nr_linux + 106;
  syscall_nr_lstat                      = __nr_linux + 107;
  syscall_nr_fstat                      = __nr_linux + 108;

  syscall_nr_unused109                   = __nr_Linux +  109;


  syscall_nr_iopl                       = __nr_Linux + 110;

  syscall_nr_vhangup                    = __nr_linux + 111;
  syscall_nr_idle                       = __nr_linux + 112;
//  syscall_nr_syscall                    = __nr_linux + 113;
  syscall_nr_vm86                       = __nr_Linux + 113;

  syscall_nr_wait4                      = __nr_linux + 114;
  syscall_nr_swapoff                    = __nr_linux + 115;
  syscall_nr_sysinfo                    = __nr_linux + 116;
  syscall_nr_ipc                        = __nr_linux + 117;
  syscall_nr_fsync                      = __nr_linux + 118;
  syscall_nr_sigreturn                  = __nr_linux + 119;
  syscall_nr_clone                      = __nr_linux + 120;
  syscall_nr_setdomainname              = __nr_linux + 121;
  syscall_nr_uname                      = __nr_linux + 122;
  syscall_nr_modify_ldt                 = __nr_linux + 123;
  syscall_nr_adjtimex                   = __nr_linux + 124;
  syscall_nr_mprotect                   = __nr_linux + 125;
  syscall_nr_sigprocmask                = __nr_linux + 126;
  syscall_nr_create_module              = __nr_linux + 127;
  syscall_nr_init_module                = __nr_linux + 128;
  syscall_nr_delete_module              = __nr_linux + 129;
  syscall_nr_get_kernel_syms            = __nr_linux + 130;
  syscall_nr_quotactl                   = __nr_linux + 131;
  syscall_nr_getpgid                    = __nr_linux + 132;
  syscall_nr_fchdir                     = __nr_linux + 133;
  syscall_nr_bdflush                    = __nr_linux + 134;
  syscall_nr_sysfs                      = __nr_linux + 135;
  syscall_nr_personality                = __nr_linux + 136;
  syscall_nr_afs_syscall                = __nr_linux + 137;

  syscall_nr_setfsuid                   = __nr_linux + 138;
  syscall_nr_setfsgid                   = __nr_linux + 139;
  syscall_nr__llseek                    = __nr_linux + 140;
  syscall_nr_getdents                   = __nr_linux + 141;
  syscall_nr__newselect                 = __nr_linux + 142;
  syscall_nr_flock                      = __nr_linux + 143;
  syscall_nr_msync                      = __nr_linux + 144;
  syscall_nr_readv                      = __nr_linux + 145;
  syscall_nr_writev                     = __nr_linux + 146;


  syscall_nr_cacheflush                 =  __nr_Linux + 147;
  syscall_nr_cachectl                   =  __nr_Linux + 148;


  syscall_nr_sysmips                    = __nr_Linux + 149;

  syscall_nr_unused150                  = __nr_Linux +  150;

  syscall_nr_getsid                     = __nr_linux +  151; // 147;
  syscall_nr_fdatasync                  = __nr_linux +  152; // 148;
  syscall_nr__sysctl                    = __nr_linux +  153; // 149;

  syscall_nr_mlock                      = __nr_linux +  154; // 150;

  syscall_nr_munlock                    = __nr_linux + 155;
  syscall_nr_mlockall                   = __nr_linux + 156;
  syscall_nr_munlockall                 = __nr_linux + 157;

  syscall_nr_sched_setparam             = __nr_linux + 158;
  syscall_nr_sched_getparam             = __nr_linux + 159;

  syscall_nr_sched_setscheduler         = __nr_linux + 160;
  syscall_nr_sched_getscheduler         = __nr_linux + 161;
  syscall_nr_sched_yield                = __nr_linux + 162;


  syscall_nr_sched_get_priority_max     = __nr_linux + 163;
  syscall_nr_sched_get_priority_min     = __nr_linux + 164;
  syscall_nr_sched_rr_get_interval      = __nr_linux + 165;

  syscall_nr_nanosleep                  = __nr_linux + 166; // 162;
  syscall_nr_mremap                     = __nr_linux + 167;

  syscall_nr_accept      =   __nr_Linux + 168;
  syscall_nr_bind        =   __nr_Linux + 169;
  syscall_nr_connect     =   __nr_Linux + 170;
  syscall_nr_getpeername =   __nr_Linux + 171;
  syscall_nr_getsockname =   __nr_Linux + 172;
  syscall_nr_getsockopt  =   __nr_Linux + 173;
  syscall_nr_listen      =   __nr_Linux + 174;
  syscall_nr_recv        =   __nr_Linux + 175;
  syscall_nr_recvfrom    =   __nr_Linux + 176;
  syscall_nr_recvmsg     =   __nr_Linux + 177;
  syscall_nr_send        =   __nr_Linux + 178;
  syscall_nr_sendmsg     =   __nr_Linux + 179;
  syscall_nr_sendto      =   __nr_Linux + 180;
  syscall_nr_setsockopt  =   __nr_Linux + 181;
  syscall_nr_shutdown    =   __nr_Linux + 182;
  syscall_nr_socket      =   __nr_Linux + 183;
  syscall_nr_socketpair  =   __nr_Linux + 184;

  syscall_nr_setresuid                  = __nr_linux + 185;
  syscall_nr_getresuid                  = __nr_linux + 186;
//  syscall_nr_vm86                       = __nr_linux + 166;
  syscall_nr_query_module               = __nr_linux + 187;
  syscall_nr_poll                       = __nr_linux + 188;
  syscall_nr_nfsservctl                 = __nr_linux + 189;
  syscall_nr_setresgid                  = __nr_linux + 190;
  syscall_nr_getresgid                  = __nr_linux + 191;
  syscall_nr_prctl                      = __nr_linux + 192; // 172;
  syscall_nr_rt_sigreturn               = __nr_linux + 193;
  syscall_nr_rt_sigaction               = __nr_linux + 194;
  syscall_nr_rt_sigprocmask             = __nr_linux + 195;
  syscall_nr_rt_sigpending              = __nr_linux + 196;
  syscall_nr_rt_sigtimedwait            = __nr_linux + 197;
  syscall_nr_rt_sigqueueinfo            = __nr_linux + 198;
  syscall_nr_rt_sigsuspend              = __nr_linux + 199;

  syscall_nr_pread64                      = __nr_linux + 200;
  syscall_nr_pwrite64                     = __nr_linux + 201;

  syscall_nr_chown                      = __nr_linux + 202; // 182;
  syscall_nr_getcwd                     = __nr_linux + 203; // 183;
  syscall_nr_capget                     = __nr_linux + 204; // 184;
  syscall_nr_capset                     = __nr_linux + 205; // 185;
  syscall_nr_sigaltstack                = __nr_linux + 206; // 186;
  syscall_nr_sendfile                   = __nr_linux + 207; // 187;


//  syscall_nr_vfork                      = __nr_linux + 190;
//  syscall_nr_ugetrlimit                 = __nr_linux + 191;

// the following are new syscall, ......
  syscall_nr_getpmsg     = __nr_Linux + 208;
  syscall_nr_putpmsg     = __nr_Linux + 209;

  syscall_nr_mmap2                      = __nr_linux + 210; // 192;
  syscall_nr_truncate64                 = __nr_linux + 211; // 193;
  syscall_nr_ftruncate64                = __nr_linux + 212; // 194;
  syscall_nr_stat64                     = __nr_linux + 213; // 195;
  syscall_nr_lstat64                    = __nr_linux + 214; // 196;
  syscall_nr_fstat64                    = __nr_linux + 215; // 197;

{
  syscall_nr_lchown32                   = __nr_linux + 198;
  syscall_nr_getuid32                   = __nr_linux + 199;
  syscall_nr_getgid32                   = __nr_linux + 200;
  syscall_nr_geteuid32                  = __nr_linux + 201;
  syscall_nr_getegid32                  = __nr_linux + 202;
  syscall_nr_setreuid32                 = __nr_linux + 203;
  syscall_nr_setregid32                 = __nr_linux + 204;
  syscall_nr_getgroups32                = __nr_linux + 205;
  syscall_nr_setgroups32                = __nr_linux + 206;
  syscall_nr_fchown32                   = __nr_linux + 207;
  syscall_nr_setresuid32                = __nr_linux + 208;
  syscall_nr_getresuid32                = __nr_linux + 209;
  syscall_nr_setresgid32                = __nr_linux + 210;
  syscall_nr_getresgid32                = __nr_linux + 211;
  syscall_nr_chown32                    = __nr_linux + 212;
  syscall_nr_setuid32                   = __nr_linux + 213;
  syscall_nr_setgid32                   = __nr_linux + 214;
  syscall_nr_setfsuid32                 = __nr_linux + 215;
  syscall_nr_setfsgid32                 = __nr_linux + 216;
 }

  syscall_nr_pivot_root                 = __nr_linux + 216; // 218;
  syscall_nr_mincore                    = __nr_linux + 217; // 219;
  syscall_nr_madvise                    = __nr_linux + 218; // 220;
  syscall_nr_getdents64                 = __nr_linux + 219; // 217;
  syscall_nr_fcntl64                    = __nr_linux + 220; // 221;
//  syscall_nr_security                   = __nr_linux + 223;

// syscall_nr_reserved221    (__nr_Linux + 221)

  syscall_nr_gettid                     = __nr_linux + 222; // 224;
  syscall_nr_readahead                  = __nr_linux + 223; // 225;
  syscall_nr_setxattr                   = __nr_linux + 224; // 226;
  syscall_nr_lsetxattr                  = __nr_linux + 225; // 227;
  syscall_nr_fsetxattr                  = __nr_linux + 226; // 228;
  syscall_nr_getxattr                   = __nr_linux + 227; // 229;
  syscall_nr_lgetxattr                  = __nr_linux + 228; // 230;
  syscall_nr_fgetxattr                  = __nr_linux + 229; // 231;
  syscall_nr_listxattr                  = __nr_linux + 230; // 232;
  syscall_nr_llistxattr                 = __nr_linux + 231; // 233;
  syscall_nr_flistxattr                 = __nr_linux + 232; // 234;
  syscall_nr_removexattr                = __nr_linux + 233; // 235;
  syscall_nr_lremovexattr               = __nr_linux + 234; // 236;
  syscall_nr_fremovexattr               = __nr_linux + 235; // 237;
  syscall_nr_tkill                      = __nr_linux + 236; // 238;
  syscall_nr_sendfile64                 = __nr_linux + 237; // 239;
  syscall_nr_futex                      = __nr_linux + 238; // 240;
  syscall_nr_sched_setaffinity          = __nr_linux + 239; // 241;
  syscall_nr_sched_getaffinity          = __nr_linux + 240; // 242;
  syscall_nr_io_setup                   = __nr_linux + 241; // 243;
  syscall_nr_io_destroy                 = __nr_linux + 242; // 244;
  syscall_nr_io_getevents               = __nr_linux + 243; // 245;
  syscall_nr_io_submit                  = __nr_linux + 244; // 246;
  syscall_nr_io_cancel                  = __nr_linux + 245; // 247;
  syscall_nr_exit_group                 = __nr_linux + 246; // 248;
  syscall_nr_lookup_dcookie             = __nr_linux + 247; // 249;
  syscall_nr_epoll_create               = __nr_linux + 248; // 250;
  syscall_nr_epoll_ctl                  = __nr_linux + 249; // 251;
  syscall_nr_epoll_wait                 = __nr_linux + 250; // 252;
  syscall_nr_remap_file_pages           = __nr_linux + 251; // 253;

  syscall_nr_set_tid_address  = __nr_Linux + 252;
  syscall_nr_restart_syscall  = __nr_Linux + 253;
  syscall_nr_fadvise64        = __nr_Linux + 254;
  syscall_nr_statfs64         = __nr_Linux + 255;
  syscall_nr_fstatfs64        = __nr_Linux + 256;
  syscall_nr_timer_create     = __nr_Linux + 257;
  syscall_nr_timer_settime    = __nr_Linux + 258;
  syscall_nr_timer_gettime    = __nr_Linux + 259;
  syscall_nr_timer_getoverrun = __nr_Linux + 260;
  syscall_nr_timer_delete     = __nr_Linux + 261;
  syscall_nr_clock_settime    = __nr_Linux + 262;
  syscall_nr_clock_gettime    = __nr_Linux + 263;
  syscall_nr_clock_getres     = __nr_Linux + 264;
  syscall_nr_clock_nanosleep  = __nr_Linux + 265;
  syscall_nr_tgkill           = __nr_Linux + 266;
  syscall_nr_utimes           = __nr_Linux + 267;
  syscall_nr_mbind            = __nr_Linux + 268;
  syscall_nr_get_mempolicy    = __nr_Linux + 269;
  syscall_nr_set_mempolicy    = __nr_Linux + 270;
  syscall_nr_mq_open          = __nr_Linux + 271;
  syscall_nr_mq_unlink        = __nr_Linux + 272;
  syscall_nr_mq_timedsend     = __nr_Linux + 273;
  syscall_nr_mq_timedreceive  = __nr_Linux + 274;
  syscall_nr_mq_notify        = __nr_Linux + 275;
  syscall_nr_mq_getsetattr    = __nr_Linux + 276;
  syscall_nr_vserver          = __nr_Linux + 277;
  syscall_nr_waitid           = __nr_Linux + 278;
//  /* syscall_nr_sys_setaltroot    (__nr_Linux + 279) */
  syscall_nr_add_key          = __nr_Linux + 280;
  syscall_nr_request_key      = __nr_Linux + 281;
  syscall_nr_keyctl           = __nr_Linux + 282;
  syscall_nr_set_thread_area  = __nr_Linux + 283;
  syscall_nr_inotify_init     = __nr_Linux + 284;
  syscall_nr_inotify_add_watch =    (__nr_Linux + 285);
  syscall_nr_inotify_rm_watch = __nr_Linux + 286;
  syscall_nr_migrate_pages    = __nr_Linux + 287;
  syscall_nr_openat           = __nr_Linux + 288;
  syscall_nr_mkdirat          = __nr_Linux + 289;
  syscall_nr_mknodat          = __nr_Linux + 290;
  syscall_nr_fchownat         = __nr_Linux + 291;
  syscall_nr_futimesat        = __nr_Linux + 292;
  syscall_nr_fstatat          = __nr_Linux + 293;
  syscall_nr_unlinkat         = __nr_Linux + 294;
  syscall_nr_renameat         = __nr_Linux + 295;
  syscall_nr_linkat           = __nr_Linux + 296;
  syscall_nr_symlinkat        = __nr_Linux + 297;
  syscall_nr_readlinkat       = __nr_Linux + 298;
  syscall_nr_fchmodat         = __nr_Linux + 299;
  syscall_nr_faccessat        = __nr_Linux + 300;
  syscall_nr_pselect6         = __nr_Linux + 301;
  syscall_nr_ppoll            = __nr_Linux + 302;
  syscall_nr_unshare          = __nr_Linux + 303;
  syscall_nr_splice           = __nr_Linux + 304;
  syscall_nr_sync_file_range  = __nr_Linux + 305;
  syscall_nr_tee              = __nr_Linux + 306;
  syscall_nr_vmsplice         = __nr_Linux + 307;
  syscall_nr_move_pages       = __nr_Linux + 308;
  syscall_nr_set_robust_list              = __nr_Linux + 309;
  syscall_nr_get_robust_list              = __nr_Linux + 310;
  syscall_nr_kexec_load                   = __nr_Linux + 311;
  syscall_nr_getcpu                       = __nr_Linux + 312;
  syscall_nr_epoll_pwait                  = __nr_Linux + 313;
  syscall_nr_ioprio_set                   = __nr_Linux + 314;
  syscall_nr_ioprio_get                   = __nr_Linux + 315;
  syscall_nr_utimensat                    = __nr_Linux + 316;
  syscall_nr_signalfd                     = __nr_Linux + 317;
  syscall_nr_timerfd                      = __nr_Linux + 318;
  syscall_nr_eventfd                      = __nr_Linux + 319;
  syscall_nr_fallocate                    = __nr_Linux + 320;
  syscall_nr_timerfd_create               = __nr_Linux + 321;
  syscall_nr_timerfd_gettime              = __nr_Linux + 322;
  syscall_nr_timerfd_settime              = __nr_Linux + 323;
  syscall_nr_signalfd4                    = __nr_Linux + 324;
  syscall_nr_eventfd2                     = __nr_Linux + 325;
  syscall_nr_epoll_create1                = __nr_Linux + 326;
  syscall_nr_dup3                         = __nr_Linux + 327;
  syscall_nr_pipe2                        = __nr_Linux + 328;
  syscall_nr_inotify_init1                = __nr_Linux + 329;
  syscall_nr_preadv                       = __nr_Linux + 330;
  syscall_nr_pwritev                      = __nr_Linux + 331;
  syscall_nr_rt_tgsigqueueinfo            = __nr_Linux + 332;
  syscall_nr_perf_event_open              = __nr_Linux + 333;
  syscall_nr_accept4                      = __nr_Linux + 334;
  syscall_nr_recvmmsg                     = __nr_Linux + 335;
  syscall_nr_fanotify_init                = __nr_Linux + 336;
  syscall_nr_fanotify_mark                = __nr_Linux + 337;
  syscall_nr_prlimit64                    = __nr_Linux + 338;
  syscall_nr_name_to_handle_at            = __nr_Linux + 339;
  syscall_nr_open_by_handle_at            = __nr_Linux + 340;
  syscall_nr_clock_adjtime                = __nr_Linux + 341;
  syscall_nr_syncfs                       = __nr_Linux + 342;
  syscall_nr_sendmmsg                     = __nr_Linux + 343;
  syscall_nr_setns                        = __nr_Linux + 344;
  syscall_nr_process_vm_readv             = __nr_Linux + 345;
  syscall_nr_process_vm_writev            = __nr_Linux + 346;
  syscall_nr_kcmp                         = __nr_Linux + 347;
  syscall_nr_finit_module                 = __nr_Linux + 348;
  syscall_nr_sched_setattr                = __nr_Linux + 349;
  syscall_nr_sched_getattr                = __nr_Linux + 350;
  syscall_nr_renameat2                    = __nr_Linux + 351;
  syscall_nr_seccomp                      = __nr_Linux + 352;
  syscall_nr_getrandom                    = __nr_Linux + 353;
  syscall_nr_memfd_create                 = __nr_Linux + 354;
  syscall_nr_bpf                          = __nr_Linux + 355;
  syscall_nr_execveat                     = __nr_Linux + 356;
  syscall_nr_userfaultfd                  = __nr_Linux + 357;
  syscall_nr_membarrier                   = __nr_Linux + 358;
  syscall_nr_mlock2                       = __nr_Linux + 359;
  syscall_nr_copy_file_range              = __nr_Linux + 360;
  syscall_nr_preadv2                      = __nr_Linux + 361;
  syscall_nr_pwritev2                     = __nr_Linux + 362;
  syscall_nr_pkey_mprotect                = __nr_Linux + 363;
  syscall_nr_pkey_alloc                   = __nr_Linux + 364;
  syscall_nr_pkey_free                    = __nr_Linux + 365;
  syscall_nr_statx                        = __nr_Linux + 366;
  syscall_nr_rseq                         = __nr_Linux + 367;
  syscall_nr_io_pgetevents                = __nr_Linux + 368;
  syscall_nr_semget                       = __nr_Linux + 393;
  syscall_nr_semctl                       = __nr_Linux + 394;
  syscall_nr_shmget                       = __nr_Linux + 395;
  syscall_nr_shmctl                       = __nr_Linux + 396;
  syscall_nr_shmat                        = __nr_Linux + 397;
  syscall_nr_shmdt                        = __nr_Linux + 398;
  syscall_nr_msgget                       = __nr_Linux + 399;
  syscall_nr_msgsnd                       = __nr_Linux + 400;
  syscall_nr_msgrcv                       = __nr_Linux + 401;
  syscall_nr_msgctl                       = __nr_Linux + 402;
  syscall_nr_clock_gettime64              = __nr_Linux + 403;
  syscall_nr_clock_settime64              = __nr_Linux + 404;
  syscall_nr_clock_adjtime64              = __nr_Linux + 405;
  syscall_nr_clock_getres_time64          = __nr_Linux + 406;
  syscall_nr_clock_nanosleep_time64       = __nr_Linux + 407;
  syscall_nr_timer_gettime64              = __nr_Linux + 408;
  syscall_nr_timer_settime64              = __nr_Linux + 409;
  syscall_nr_timerfd_gettime64            = __nr_Linux + 410;
  syscall_nr_timerfd_settime64            = __nr_Linux + 411;
  syscall_nr_utimensat_time64             = __nr_Linux + 412;
  syscall_nr_pselect6_time64              = __nr_Linux + 413;
  syscall_nr_ppoll_time64                 = __nr_Linux + 414;
  syscall_nr_io_pgetevents_time64         = __nr_Linux + 416;
  syscall_nr_recvmmsg_time64              = __nr_Linux + 417;
  syscall_nr_mq_timedsend_time64          = __nr_Linux + 418;
  syscall_nr_mq_timedreceive_time64       = __nr_Linux + 419;
  syscall_nr_semtimedop_time64            = __nr_Linux + 420;
  syscall_nr_rt_sigtimedwait_time64       = __nr_Linux + 421;
  syscall_nr_futex_time64                 = __nr_Linux + 422;
  syscall_nr_sched_rr_get_interval_time64 = __nr_Linux + 423;
  syscall_nr_pidfd_send_signal            = __nr_Linux + 424;
  syscall_nr_io_uring_setup               = __nr_Linux + 425;
  syscall_nr_io_uring_enter               = __nr_Linux + 426;
  syscall_nr_io_uring_register            = __nr_Linux + 427;
  syscall_nr_open_tree                    = __nr_Linux + 428;
  syscall_nr_move_mount                   = __nr_Linux + 429;
  syscall_nr_fsopen                       = __nr_Linux + 430;
  syscall_nr_fsconfig                     = __nr_Linux + 431;
  syscall_nr_fsmount                      = __nr_Linux + 432;
  syscall_nr_fspick                       = __nr_Linux + 433;
  syscall_nr_pidfd_open                   = __nr_Linux + 434;
  syscall_nr_clone3                       = __nr_Linux + 435;

