{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    Syscall nrs for x86-64

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{
* This file contains the system call numbers.
}
const
  syscall_nr_read                                = 0;
  syscall_nr_write                               = 1;
  syscall_nr_open                                = 2;
  syscall_nr_close                               = 3;
  syscall_nr_stat                                = 4;
  syscall_nr_fstat                               = 5;
  syscall_nr_lstat                               = 6;
  syscall_nr_poll                                = 7;
  syscall_nr_lseek                               = 8;
  syscall_nr_mmap                                = 9;
  syscall_nr_mprotect                           = 10;
  syscall_nr_munmap                             = 11;
  syscall_nr_brk                                = 12;
  syscall_nr_rt_sigaction                       = 13;
  syscall_nr_rt_sigprocmask                     = 14;
  syscall_nr_rt_sigreturn                       = 15;
  syscall_nr_ioctl                              = 16;
  syscall_nr_pread64                            = 17;
  syscall_nr_pwrite64                           = 18;
  syscall_nr_readv                              = 19;
  syscall_nr_writev                             = 20;
  syscall_nr_access                             = 21;
  syscall_nr_pipe                               = 22;
  syscall_nr_select                             = 23;
  syscall_nr_sched_yield                        = 24;
  syscall_nr_mremap                             = 25;
  syscall_nr_msync                              = 26;
  syscall_nr_mincore                            = 27;
  syscall_nr_madvise                            = 28;
  syscall_nr_shmget                             = 29;
  syscall_nr_shmat                              = 30;
  syscall_nr_shmctl                             = 31;
  syscall_nr_dup                                = 32;
  syscall_nr_dup2                               = 33;
  syscall_nr_pause                              = 34;
  syscall_nr_nanosleep                          = 35;
  syscall_nr_getitimer                          = 36;
  syscall_nr_alarm                              = 37;
  syscall_nr_setitimer                          = 38;
  syscall_nr_getpid                             = 39;
  syscall_nr_sendfile                           = 40;
  syscall_nr_socket                             = 41;
  syscall_nr_connect                            = 42;
  syscall_nr_accept                             = 43;
  syscall_nr_sendto                             = 44;
  syscall_nr_recvfrom                           = 45;
  syscall_nr_sendmsg                            = 46;
  syscall_nr_recvmsg                            = 47;

  syscall_nr_shutdown                           = 48;
  syscall_nr_bind                               = 49;
  syscall_nr_listen                             = 50;
  syscall_nr_getsockname                        = 51;
  syscall_nr_getpeername                        = 52;
  syscall_nr_socketpair                         = 53;
  syscall_nr_setsockopt                         = 54;
  syscall_nr_getsockopt                         = 55;

  syscall_nr_clone                              = 56;
  syscall_nr_fork                               = 57;
  syscall_nr_vfork                              = 58;
  syscall_nr_execve                             = 59;
  syscall_nr_exit                               = 60;
  syscall_nr_wait4                              = 61;
  syscall_nr_kill                               = 62;
  syscall_nr_uname                              = 63;

  syscall_nr_semget                             = 64;
  syscall_nr_semop                              = 65;
  syscall_nr_semctl                             = 66;
  syscall_nr_shmdt                              = 67;
  syscall_nr_msgget                             = 68;
  syscall_nr_msgsnd                             = 69;
  syscall_nr_msgrcv                             = 70;
  syscall_nr_msgctl                             = 71;

  syscall_nr_fcntl                              = 72;
  syscall_nr_flock                              = 73;
  syscall_nr_fsync                              = 74;
  syscall_nr_fdatasync                          = 75;
  syscall_nr_truncate                           = 76;
  syscall_nr_ftruncate                          = 77;
  syscall_nr_getdents                           = 78;
  syscall_nr_getcwd                             = 79;

  syscall_nr_chdir                              = 80;
  syscall_nr_fchdir                             = 81;
  syscall_nr_rename                             = 82;
  syscall_nr_mkdir                              = 83;
  syscall_nr_rmdir                              = 84;
  syscall_nr_creat                              = 85;
  syscall_nr_link                               = 86;
  syscall_nr_unlink                             = 87;

  syscall_nr_symlink                            = 88;
  syscall_nr_readlink                           = 89;
  syscall_nr_chmod                              = 90;
  syscall_nr_fchmod                             = 91;
  syscall_nr_chown                              = 92;
  syscall_nr_fchown                             = 93;
  syscall_nr_lchown                             = 94;
  syscall_nr_umask                              = 95;

  syscall_nr_gettimeofday                       = 96;
  syscall_nr_getrlimit                          = 97;
  syscall_nr_getrusage                          = 98;
  syscall_nr_sysinfo                            = 99;
  syscall_nr_times                             = 100;
  syscall_nr_ptrace                            = 101;
  syscall_nr_getuid                            = 102;
  syscall_nr_syslog                            = 103;

  syscall_nr_getgid                            = 104;
  syscall_nr_setuid                            = 105;
  syscall_nr_setgid                            = 106;
  syscall_nr_geteuid                           = 107;
  syscall_nr_getegid                           = 108;
  syscall_nr_setpgid                           = 109;
  syscall_nr_getppid                           = 110;
  syscall_nr_getpgrp                           = 111;

  syscall_nr_setsid                            = 112;
  syscall_nr_setreuid                          = 113;
  syscall_nr_setregid                          = 114;
  syscall_nr_getgroups                         = 115;
  syscall_nr_setgroups                         = 116;
  syscall_nr_setresuid                         = 117;
  syscall_nr_getresuid                         = 118;
  syscall_nr_setresgid                         = 119;

  syscall_nr_getresgid                         = 120;
  syscall_nr_getpgid                           = 121;
  syscall_nr_setfsuid                          = 122;
  syscall_nr_setfsgid                          = 123;
  syscall_nr_getsid                            = 124;
  syscall_nr_capget                            = 125;
  syscall_nr_capset                            = 126;

  syscall_nr_rt_sigpending                     = 127;
  syscall_nr_rt_sigtimedwait                   = 128;
  syscall_nr_rt_sigqueueinfo                   = 129;
  syscall_nr_rt_sigsuspend                     = 130;
  syscall_nr_sigaltstack                       = 131;
  syscall_nr_utime                             = 132;
  syscall_nr_mknod                             = 133;

  syscall_nr_uselib                            = 134;
  syscall_nr_personality                       = 135;

  syscall_nr_ustat                             = 136;
  syscall_nr_statfs                            = 137;
  syscall_nr_fstatfs                           = 138;
  syscall_nr_sysfs                             = 139;

  syscall_nr_getpriority                       = 140;
  syscall_nr_setpriority                       = 141;
  syscall_nr_sched_setparam                    = 142;
  syscall_nr_sched_getparam                    = 143;
  syscall_nr_sched_setscheduler                = 144;
  syscall_nr_sched_getscheduler                = 145;
  syscall_nr_sched_get_priority_max            = 146;
  syscall_nr_sched_get_priority_min            = 147;
  syscall_nr_sched_rr_get_interval             = 148;

  syscall_nr_mlock                             = 149;
  syscall_nr_munlock                           = 150;
  syscall_nr_mlockall                          = 151;
  syscall_nr_munlockall                        = 152;

  syscall_nr_vhangup                           = 153;

  syscall_nr_modify_ldt                        = 154;

  syscall_nr_pivot_root                        = 155;

  syscall_nr__sysctl                           = 156;

  syscall_nr_prctl                             = 157;
  syscall_nr_arch_prctl                        = 158;

  syscall_nr_adjtimex                          = 159;

  syscall_nr_setrlimit                         = 160;

  syscall_nr_chroot                            = 161;

  syscall_nr_sync                              = 162;

  syscall_nr_acct                              = 163;

  syscall_nr_settimeofday                      = 164;

  syscall_nr_mount                             = 165;
  syscall_nr_umount2                           = 166;

  syscall_nr_swapon                            = 167;
  syscall_nr_swapoff                           = 168;

  syscall_nr_reboot                            = 169;

  syscall_nr_sethostname                       = 170;
  syscall_nr_setdomainname                     = 171;

  syscall_nr_iopl                              = 172;
  syscall_nr_ioperm                            = 173;

  syscall_nr_create_module                     = 174;
  syscall_nr_init_module                       = 175;
  syscall_nr_delete_module                     = 176;
  syscall_nr_get_kernel_syms                   = 177;
  syscall_nr_query_module                      = 178;

  syscall_nr_quotactl                          = 179;

  syscall_nr_nfsservctl                        = 180;

  syscall_nr_getpmsg                           = 181;
  syscall_nr_putpmsg                           = 182;

  syscall_nr_afs_syscall                       = 183;

  syscall_nr_tuxcall                    = 184;

  syscall_nr_security                   = 185;

  syscall_nr_gettid             = 186;

  syscall_nr_readahead          = 187;
  syscall_nr_setxattr           = 188;
  syscall_nr_lsetxattr          = 189;
  syscall_nr_fsetxattr          = 190;
  syscall_nr_getxattr           = 191;
  syscall_nr_lgetxattr          = 192;
  syscall_nr_fgetxattr          = 193;
  syscall_nr_listxattr          = 194;
  syscall_nr_llistxattr         = 195;
  syscall_nr_flistxattr         = 196;
  syscall_nr_removexattr        = 197;
  syscall_nr_lremovexattr       = 198;
  syscall_nr_fremovexattr       = 199;
  syscall_nr_tkill      = 200;
  syscall_nr_time      = 201;
  syscall_nr_futex     = 202;
  syscall_nr_sched_setaffinity    = 203;
  syscall_nr_sched_getaffinity     = 204;
  syscall_nr_set_thread_area    = 205;
  syscall_nr_io_setup   = 206;
  syscall_nr_io_destroy = 207;
  syscall_nr_io_getevents       = 208;
  syscall_nr_io_submit  = 209;
  syscall_nr_io_cancel  = 210;
  syscall_nr_get_thread_area    = 211;
  syscall_nr_lookup_dcookie     = 212;
  syscall_nr_epoll_create       = 213;
  syscall_nr_epoll_ctl_old      = 214;
  syscall_nr_epoll_wait_old     = 215;
  syscall_nr_remap_file_pages   = 216;
  syscall_nr_getdents64 = 217;
  syscall_nr_set_tid_address    = 218;
  syscall_nr_restart_syscall    = 219;
  syscall_nr_semtimedop         = 220;
  syscall_nr_fadvise64          = 221;
  syscall_nr_timer_create       = 222;
  syscall_nr_timer_settime      = 223;
  syscall_nr_timer_gettime      = 224;
  syscall_nr_timer_getoverrun   = 225;
  syscall_nr_timer_delete       = 226;
  syscall_nr_clock_settime      = 227;
  syscall_nr_clock_gettime      = 228;
  syscall_nr_clock_getres       = 229;
  syscall_nr_clock_nanosleep    = 230;
  syscall_nr_exit_group         = 231;
  syscall_nr_epoll_wait         = 232;
  syscall_nr_epoll_ctl          = 233;
  syscall_nr_tgkill             = 234;
  syscall_nr_utimes             = 235;
  syscall_nr_vserver            = 236;
  syscall_nr_mbind              = 237;
  syscall_nr_set_mempolicy      = 238;
  syscall_nr_get_mempolicy      = 239;
  syscall_nr_mq_open            = 240;
  syscall_nr_mq_unlink          = 241;
  syscall_nr_mq_timedsend       = 242;
  syscall_nr_mq_timedreceive    = 243;
  syscall_nr_mq_notify          = 244;
  syscall_nr_mq_getsetattr      = 245;
  syscall_nr_kexec_load         = 246;
  syscall_nr_waitid             = 247;
  syscall_nr_add_key            = 248;
  syscall_nr_request_key        = 249;
  syscall_nr_keyctl             = 250;
  syscall_nr_ioprio_set         = 251;
  syscall_nr_ioprio_get         = 252;
  syscall_nr_inotify_init       = 253;
  syscall_nr_inotify_add_watch  = 254;
  syscall_nr_inotify_rm_watch   = 255;

  syscall_nr_migrate_pages      = 256;
  syscall_nr_openat             = 257;
  syscall_nr_mkdirat            = 258;
  syscall_nr_mknodat            = 259;
  syscall_nr_fchownat           = 260;
  syscall_nr_futimesat          = 261;
  syscall_nr_newfstatat         = 262;
  syscall_nr_unlinkat           = 263;
  syscall_nr_renameat           = 264;
  syscall_nr_linkat             = 265;
  syscall_nr_symlinkat          = 266;
  syscall_nr_readlinkat         = 267;
  syscall_nr_fchmodat           = 268;
  syscall_nr_faccessat          = 269;
  syscall_nr_pselect6           = 270;
  syscall_nr_ppoll              = 271;
  syscall_nr_unshare            = 272;
  syscall_nr_set_robust_list    = 273;
  syscall_nr_get_robust_list    = 274;
  syscall_nr_splice             = 275;
  syscall_nr_tee                = 276;
  syscall_nr_sync_file_range    = 277;
  syscall_nr_vmsplice           = 278;
  syscall_nr_move_pages         = 279;
  syscall_nr_utimensat          = 280;
  syscall_nr_epoll_pwait        = 281;
  syscall_nr_signalfd           = 282;
  syscall_nr_timerfd_create     = 283;
  syscall_nr_eventfd            = 284;
  syscall_nr_fallocate          = 285;
  syscall_nr_timerfd_settime    = 286;
  syscall_nr_timerfd_gettime    = 287;
  syscall_nr_accept4            = 288;
  syscall_nr_signalfd4          = 289;
  syscall_nr_eventfd2           = 290;
  syscall_nr_epoll_create1      = 291;
  syscall_nr_dup3               = 292;
  syscall_nr_pipe2              = 293;
  syscall_nr_inotify_init1      = 294;
  syscall_nr_preadv             = 295;
  syscall_nr_pwritev            = 296;
  syscall_nr_rt_tgsigqueueinfo  = 297;
  syscall_nr_perf_event_open    = 298;
  syscall_nr_recvmmsg           = 299;
  syscall_nr_fanotify_init      = 300;
  syscall_nr_fanotify_mark      = 301;
  syscall_nr_prlimit64          = 302;
  syscall_nr_name_to_handle_at  = 303;
  syscall_nr_open_by_handle_at  = 304;
  syscall_nr_clock_adjtime      = 305;
  syscall_nr_syncfs             = 306;
  syscall_nr_sendmmsg           = 307;
  syscall_nr_setns              = 308;
  syscall_nr_getcpu             = 309;
  syscall_nr_process_vm_readv   = 310;
  syscall_nr_process_vm_writev  = 311;
  syscall_nr_kcmp               = 312;
  syscall_nr_finit_module       = 313;
  syscall_nr_sched_setattr      = 314;
  syscall_nr_sched_getattr      = 315;
  syscall_nr_renameat2          = 316;
  syscall_nr_seccomp            = 317;
  syscall_nr_getrandom          = 318;
  syscall_nr_memfd_create       = 319;
  syscall_nr_kexec_file_load    = 320;
  syscall_nr_bpf                = 321;
  syscall_nr_userfaultfd        = 323;
  syscall_nr_membarrier         = 324;
  syscall_nr_mlock2             = 325;
  syscall_nr_copy_file_range    = 326;
  syscall_nr_pkey_mprotect      = 329;
  syscall_nr_pkey_alloc         = 330;
  syscall_nr_pkey_free          = 331;
  syscall_nr_statx                        = 332;
  syscall_nr_io_pgetevents                = 333;
  syscall_nr_rseq                         = 334;
  syscall_nr_pidfd_send_signal            = 424;
  syscall_nr_io_uring_setup               = 425;
  syscall_nr_io_uring_enter               = 426;
  syscall_nr_io_uring_register            = 427;
  syscall_nr_open_tree                    = 428;
  syscall_nr_move_mount                   = 429;
  syscall_nr_fsopen                       = 430;
  syscall_nr_fsconfig                     = 431;
  syscall_nr_fsmount                      = 432;
  syscall_nr_fspick                       = 433;
  syscall_nr_pidfd_open                   = 434;
  syscall_nr_clone3                       = 435;

