{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by Peter Vreman
    member of the Free Pascal development team.

    Syscall nrs for Sparc

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
      syscall_nr_restart_syscall    = 0 ; // Linux Specific
      syscall_nr_exit               = 1 ; // Common
      syscall_nr_fork               = 2 ; // Common
      syscall_nr_read               = 3 ; // Common
      syscall_nr_write              = 4 ; // Common
      syscall_nr_open               = 5 ; // Common
      syscall_nr_close              = 6 ; // Common
      syscall_nr_wait4              = 7 ; // Common
      syscall_nr_creat              = 8 ; // Common
      syscall_nr_link               = 9 ; // Common
      syscall_nr_unlink            = 10 ; // Common
      syscall_nr_execv             = 11 ; // SunOS Specific
      syscall_nr_chdir             = 12 ; // Common
      syscall_nr_chown             = 13 ; // Common
      syscall_nr_mknod             = 14 ; // Common
      syscall_nr_chmod             = 15 ; // Common
      syscall_nr_lchown            = 16 ; // Common
      syscall_nr_brk               = 17 ; // Common
      syscall_nr_perfctr           = 18 ; // Performance counter operations
      syscall_nr_lseek             = 19 ; // Common
      syscall_nr_getpid            = 20 ; // Common
      syscall_nr_capget            = 21 ; // Linux Specific
      syscall_nr_capset            = 22 ; // Linux Specific
      syscall_nr_setuid            = 23 ; // Implemented via setreuid in SunOS
      syscall_nr_getuid            = 24 ; // Common
      syscall_nr_vmsplice          = 25 ; // ENOSYS under SunOS
      syscall_nr_ptrace            = 26 ; // Common
      syscall_nr_alarm             = 27 ; // Implemented via setitimer in SunOS
      syscall_nr_sigaltstack       = 28 ; // Common
      syscall_nr_pause             = 29 ; // Is sigblock(0)->sigpause() in SunOS
      syscall_nr_utime             = 30 ; // Implemented via utimes() under SunOS
{$ifndef CPUSPARC64}
      syscall_nr_lchown32          = 31 ; // Linux sparc32 specific
      syscall_nr_fchown32          = 32 ; // Linux sparc32 specific
{$endif ndef CPUSPARC64}
      syscall_nr_access            = 33 ; // Common
      syscall_nr_nice              = 34 ; // Implemented via get/setpriority() in SunOS
{$ifndef CPUSPARC64}
      syscall_nr_chown32           = 35 ; // Linux sparc32 specific
{$endif ndef CPUSPARC64}
      syscall_nr_sync              = 36 ; // Common
      syscall_nr_kill              = 37 ; // Common
      syscall_nr_stat              = 38 ; // Common
      syscall_nr_sendfile          = 39 ; // Linux Specific
      syscall_nr_lstat             = 40 ; // Common
      syscall_nr_dup               = 41 ; // Common
      syscall_nr_pipe              = 42 ; // Common
      syscall_nr_times             = 43 ; // Implemented via getrusage() in SunOS
{$ifndef CPUSPARC64}
      syscall_nr_getuid32          = 44 ; // Linux sparc32 specific
{$endif ndef CPUSPARC64}
      syscall_nr_umount2           = 45 ; // Linux Specific
      syscall_nr_setgid            = 46 ; // Implemented via setregid() in SunOS
      syscall_nr_getgid            = 47 ; // Common
      syscall_nr_signal            = 48 ; // Implemented via sigvec() in SunOS
      syscall_nr_geteuid           = 49 ; // SunOS calls getuid()
      syscall_nr_getegid           = 50 ; // SunOS calls getgid()
      syscall_nr_acct              = 51 ; // Common
//       syscall_nr_memory_ordering= 52    Linux sparc64 specific
{$ifndef CPUSPARC64}
      syscall_nr_getgid32          = 53 ; // Linux sparc32 specific
{$endif ndef CPUSPARC64}
      syscall_nr_ioctl             = 54 ; // Common
      syscall_nr_reboot            = 55 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_mmap2             = 56 ; // Linux sparc32 Specific
{$endif ndef CPUSPARC64}
      syscall_nr_symlink           = 57 ; // Common
      syscall_nr_readlink          = 58 ; // Common
      syscall_nr_execve            = 59 ; // Common
      syscall_nr_umask             = 60 ; // Common
      syscall_nr_chroot            = 61 ; // Common
      syscall_nr_fstat             = 62 ; // Common
      syscall_nr_fstat64           = 63 ; // Linux sparc32 Specific
      syscall_nr_getpagesize       = 64 ; // Common
      syscall_nr_msync             = 65 ; // Common in newer 1.3.x revs...
      syscall_nr_vfork             = 66 ; // Common
      syscall_nr_pread64           = 67 ; // Linux Specific
      syscall_nr_pwrite64          = 68 ; // Linux Specific
{$ifndef CPUSPARC64}
      syscall_nr_geteuid32         = 69 ; // Linux sparc32, sbrk under SunOS
      syscall_nr_getegid32         = 70 ; // Linux sparc32, sstk under SunOS
{$endif ndef CPUSPARC64}
      syscall_nr_mmap              = 71 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_setreuid32        = 72 ; // Linux sparc32, vadvise under SunOS
{$endif ndef CPUSPARC64}
      syscall_nr_munmap            = 73 ; // Common
      syscall_nr_mprotect          = 74 ; // Common
      syscall_nr_madvise           = 75 ; // Common
      syscall_nr_vhangup           = 76 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_truncate64        = 77 ; // Linux sparc32 Specific
{$endif ndef CPUSPARC64}
      syscall_nr_mincore           = 78 ; // Common
      syscall_nr_getgroups         = 79 ; // Common
      syscall_nr_setgroups         = 80 ; // Common
      syscall_nr_getpgrp           = 81 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_setgroups32       = 82 ; // Linux sparc32, setpgrp under SunOS
{$endif ndef CPUSPARC64}
      syscall_nr_setitimer         = 83 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_ftruncate64       = 84 ; // Linux sparc32 Specific
{$endif ndef CPUSPARC64}
      syscall_nr_swapon            = 85 ; // Common
      syscall_nr_getitimer         = 86 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_setuid32          = 87 ; // Linux sparc32, gethostname under SunOS
{$endif ndef CPUSPARC64}
      syscall_nr_sethostname       = 88 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_setgid32          = 89 ; // Linux sparc32, getdtablesize under SunOS
{$endif ndef CPUSPARC64}
      syscall_nr_dup2              = 90 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_setfsuid32        = 91 ; // Linux sparc32, getdopt under SunOS
{$endif ndef CPUSPARC64}
      syscall_nr_fcntl             = 92 ; // Common
      syscall_nr_select            = 93 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_setfsgid32        = 94 ; // Linux sparc32, setdopt under SunOS
{$endif ndef CPUSPARC64}
      syscall_nr_fsync             = 95 ; // Common
      syscall_nr_setpriority       = 96 ; // Common
      syscall_nr_socket            = 97 ; // Common
      syscall_nr_connect           = 98 ; // Common
      syscall_nr_accept            = 99 ; // Common
      syscall_nr_getpriority      = 100 ; // Common
      syscall_nr_rt_sigreturn     = 101 ; // Linux Specific
      syscall_nr_rt_sigaction     = 102 ; // Linux Specific
      syscall_nr_rt_sigprocmask   = 103 ; // Linux Specific
      syscall_nr_rt_sigpending    = 104 ; // Linux Specific
      syscall_nr_rt_sigtimedwait  = 105 ; // Linux Specific
      syscall_nr_rt_sigqueueinfo  = 106 ; // Linux Specific
      syscall_nr_rt_sigsuspend    = 107 ; // Linux Specific
{$ifndef CPUSPARC64}
      syscall_nr_setresuid32      = 108 ; // Linux Specific, sigvec under SunOS
      syscall_nr_getresuid32      = 109 ; // Linux Specific, sigblock under SunOS
      syscall_nr_setresgid32      = 110 ; // Linux Specific, sigsetmask under SunOS
      syscall_nr_getresgid32      = 111 ; // Linux Specific, sigpause under SunOS
      syscall_nr_setregid32       = 112 ; // Linux sparc32, sigstack under SunOS
{$else}
      syscall_nr_setresuid        = 108 ; // Linux Specific, sigvec under SunOS
      syscall_nr_getresuid        = 109 ; // Linux Specific, sigblock under SunOS
      syscall_nr_setresgid        = 110 ; // Linux Specific, sigsetmask under SunOS
      syscall_nr_getresgid        = 111 ; // Linux Specific, sigpause under SunOS
{$endif}
      syscall_nr_recvmsg          = 113 ; // Common
      syscall_nr_sendmsg          = 114 ; // Common
{$ifndef CPUSPARC64}
      syscall_nr_getgroups32      = 115 ; // Linux sparc32, vtrace under SunOS
{$endif ndef CPUSPARC64}
      syscall_nr_gettimeofday     = 116 ; // Common
      syscall_nr_getrusage        = 117 ; // Common
      syscall_nr_getsockopt       = 118 ; // Common
      syscall_nr_getcwd           = 119 ; // Linux Specific
      syscall_nr_readv            = 120 ; // Common
      syscall_nr_writev           = 121 ; // Common
      syscall_nr_settimeofday     = 122 ; // Common
      syscall_nr_fchown           = 123 ; // Common
      syscall_nr_fchmod           = 124 ; // Common
      syscall_nr_recvfrom         = 125 ; // Common
      syscall_nr_setreuid         = 126 ; // Common
      syscall_nr_setregid         = 127 ; // Common
      syscall_nr_rename           = 128 ; // Common
      syscall_nr_truncate         = 129 ; // Common
      syscall_nr_ftruncate        = 130 ; // Common
      syscall_nr_flock            = 131 ; // Common
      syscall_nr_lstat64          = 132 ; // Linux sparc32 Specific
      syscall_nr_sendto           = 133 ; // Common
      syscall_nr_shutdown         = 134 ; // Common
      syscall_nr_socketpair       = 135 ; // Common
      syscall_nr_mkdir            = 136 ; // Common
      syscall_nr_rmdir            = 137 ; // Common
      syscall_nr_utimes           = 138 ; // SunOS Specific
      syscall_nr_stat64           = 139 ; // Linux sparc32 Specific
      syscall_nr_sendfile64       = 140 ; // adjtime under SunOS
      syscall_nr_getpeername      = 141 ; // Common
      syscall_nr_futex            = 142 ; // gethostid under SunOS
      syscall_nr_gettid           = 143 ; // ENOSYS under SunOS
      syscall_nr_getrlimit        = 144 ; // Common
      syscall_nr_setrlimit        = 145 ; // Common
      syscall_nr_pivot_root       = 146 ; // Linux Specific, killpg under SunOS
      syscall_nr_prctl            = 147 ; // ENOSYS under SunOS
      syscall_nr_pciconfig_read   = 148 ; // ENOSYS under SunOS
      syscall_nr_pciconfig_write  = 149 ; // ENOSYS under SunOS
      syscall_nr_getsockname      = 150 ; // Common
      syscall_nr_inotify_init     = 151 ; // Linux specific
      syscall_nr_inotify_add_watch= 152 ; // Linux specific
      syscall_nr_poll             = 153 ; // Common
      syscall_nr_getdents64       = 154 ; // Linux specific
{$ifndef CPUSPARC64}
      syscall_nr_fcntl64          = 155 ; // Linux sparc32 Specific
{$endif ndef CPUSPARC64}
      syscall_nr_inotify_rm_watch = 156 ; // Linux specific
      syscall_nr_statfs           = 157 ; // Common
      syscall_nr_fstatfs          = 158 ; // Common
      syscall_nr_umount           = 159 ; // Common
      syscall_nr_sched_set_affinity = 160; // Linux specific, async_daemon under SunOS
      syscall_nr_sched_get_affinity = 161; // Linux specific, getfh under SunOS
      syscall_nr_getdomainname    = 162 ; // SunOS Specific
      syscall_nr_setdomainname    = 163 ; // Common
{$ifdef CPUSPARC64}
      syscall_nr_utrap_install    = 164 ; // SYSV ABI/v9 required
{$endif def CPUSPARC64}
      syscall_nr_quotactl         = 165 ; // Common
      syscall_nr_set_tid_address  = 166 ; // Linux specific, exportfs under SunOS
      syscall_nr_mount            = 167 ; // Common
      syscall_nr_ustat            = 168 ; // Common
      syscall_nr_setxattr         = 169 ; // SunOS: semsys
      syscall_nr_lsetxattr        = 170 ; // SunOS: msgsys
      syscall_nr_fsetxattr        = 171 ; // SunOS: shmsys
      syscall_nr_getxattr         = 172 ; // SunOS: auditsys
      syscall_nr_lgetxattr        = 173 ; // SunOS: rfssys
      syscall_nr_getdents         = 174 ; // Common
      syscall_nr_setsid           = 175 ; // Common
      syscall_nr_fchdir           = 176 ; // Common
      syscall_nr_fgetxattr        = 177 ; // SunOS: fchroot
      syscall_nr_listxattr        = 178 ; // SunOS: vpixsys
      syscall_nr_llistxattr       = 179 ; // SunOS: aioread
      syscall_nr_flistxattr       = 180 ; // SunOS: aiowrite
      syscall_nr_removexattr      = 181 ; // SunOS: aiowait
      syscall_nr_lremovexattr     = 182 ; // SunOS: aiocancel
      syscall_nr_sigpending       = 183 ; // Common
      syscall_nr_query_module     = 184 ; // Linux Specific
      syscall_nr_setpgid          = 185 ; // Common
      syscall_nr_fremovexattr     = 186 ; // SunOS: pathconf
      syscall_nr_tkill            = 187 ; // SunOS: fpathconf
      syscall_nr_exit_group       = 188 ; // SunOS sysconf
      syscall_nr_uname            = 189 ; // Linux Specific
      syscall_nr_init_module      = 190 ; // Linux Specific
      syscall_nr_personality      = 191 ; // Linux Specific
      syscall_nr_remap_file_pages = 192 ; // Linux Specific
      syscall_nr_epoll_create     = 193 ; // Linux Specific - was break
      syscall_nr_epoll_ctl        = 194 ; // Linux Specific - was lock
      syscall_nr_epoll_wait       = 195 ; // Linux Specific - was mpx
      syscall_nr_ioprio_set       = 196 ; // Linux Specific
      syscall_nr_getppid          = 197 ; // Linux Specific
      syscall_nr_sigaction        = 198 ; // Linux Specific
      syscall_nr_sgetmask         = 199 ; // Linux Specific
      syscall_nr_ssetmask         = 200 ; // Linux Specific
      syscall_nr_sigsuspend       = 201 ; // Linux Specific
      syscall_nr_oldlstat         = 202 ; // Linux Specific
      syscall_nr_uselib           = 203 ; // Linux Specific
      syscall_nr_readdir          = 204 ; // Linux Specific
      syscall_nr_readahead        = 205 ; // Linux Specific
      syscall_nr_socketcall       = 206 ; // Linux Specific
      syscall_nr_syslog           = 207 ; // Linux Specific
      syscall_nr_lookup_dcookie   = 208 ; // Linux Specific
      syscall_nr_fadvise64        = 209 ; // Linux Specific
      syscall_nr_fadvise64_64     = 210 ; // Linux Specific
      syscall_nr_tgkill           = 211 ; // Linux Specific
      syscall_nr_waitpid          = 212 ; // Linux Specific
      syscall_nr_swapoff          = 213 ; // Linux Specific
      syscall_nr_sysinfo          = 214 ; // Linux Specific
      syscall_nr_ipc              = 215 ; // Linux Specific
      syscall_nr_sigreturn        = 216 ; // Linux Specific
      syscall_nr_clone            = 217 ; // Linux Specific
      syscall_nr_ioprio_get       = 218 ; // Linux Specific
      syscall_nr_adjtimex         = 219 ; // Linux Specific
      syscall_nr_sigprocmask      = 220 ; // Linux Specific
      syscall_nr_create_module    = 221 ; // Linux Specific
      syscall_nr_delete_module    = 222 ; // Linux Specific
      syscall_nr_get_kernel_syms  = 223 ; // Linux Specific
      syscall_nr_getpgid          = 224 ; // Linux Specific
      syscall_nr_bdflush          = 225 ; // Linux Specific
      syscall_nr_sysfs            = 226 ; // Linux Specific
      syscall_nr_afs_syscall      = 227 ; // Linux Specific
      syscall_nr_setfsuid         = 228 ; // Linux Specific
      syscall_nr_setfsgid         = 229 ; // Linux Specific
      syscall_nr__newselect       = 230 ; // Linux Specific
{$ifndef CPUSPARC64}
      syscall_nr_time             = 231 ; // Linux Specific
{$endif ndef CPUSPARC64}
      syscall_nr_splice           = 232 ; // Linux Specific
      syscall_nr_stime            = 233 ; // Linux Specific
      syscall_nr_statfs64         = 234 ; // Linux Specific
      syscall_nr_fstatfs64        = 235 ; // Linux Specific
      syscall_nr__llseek          = 236 ; // Linux Specific
      syscall_nr_mlock            = 237;
      syscall_nr_munlock          = 238;
      syscall_nr_mlockall         = 239;
      syscall_nr_munlockall       = 240;
      syscall_nr_sched_setparam   = 241;
      syscall_nr_sched_getparam   = 242;
      syscall_nr_sched_setscheduler = 243;
      syscall_nr_sched_getscheduler = 244;
      syscall_nr_sched_yield      = 245;
      syscall_nr_sched_get_priority_max = 246;
      syscall_nr_sched_get_priority_min = 247;
      syscall_nr_sched_rr_get_interval= 248;
      syscall_nr_nanosleep        = 249;
      syscall_nr_mremap           = 250;
      syscall_nr__sysctl          = 251;
      syscall_nr_getsid           = 252;
      syscall_nr_fdatasync        = 253;
      syscall_nr_nfsservctl       = 254;
      syscall_nr_sync_file_range   = 255;
      syscall_nr_clock_settime     = 256;
      syscall_nr_clock_gettime     = 257;
      syscall_nr_clock_getres      = 258;
      syscall_nr_clock_nanosleep   = 259;
      syscall_nr_sched_getaffinity = 260;
      syscall_nr_sched_setaffinity = 261;
      syscall_nr_timer_settime     = 262;
      syscall_nr_timer_gettime     = 263;
      syscall_nr_timer_getoverrun  = 264;
      syscall_nr_timer_delete      = 265;
      syscall_nr_timer_create      = 266;
{$ifdef VSERVER}
      syscall_nr_vserver           = 267; //  Reserved for VSERVER
{$endif def VSERVER}
      syscall_nr_io_setup          = 268;
      syscall_nr_io_destroy        = 269;
      syscall_nr_io_submit         = 270;
      syscall_nr_io_cancel         = 271;
      syscall_nr_io_getevents      = 272;
      syscall_nr_mq_open           = 273;
      syscall_nr_mq_unlink         = 274;
      syscall_nr_mq_timedsend      = 275;
      syscall_nr_mq_timedreceive   = 276;
      syscall_nr_mq_notify         = 277;
      syscall_nr_mq_getsetattr     = 278;
      syscall_nr_waitid            = 279;
      syscall_nr_tee               = 280; { was syscall_nr_sys_setaltroot    280; }
      syscall_nr_add_key           = 281;
      syscall_nr_request_key       = 282;
      syscall_nr_keyctl            = 283;
      syscall_nr_openat            = 284;
      syscall_nr_mkdirat           = 285;
      syscall_nr_mknodat           = 286;
      syscall_nr_fchownat          = 287;
      syscall_nr_futimesat         = 288;
      syscall_nr_fstatat64         = 289;
      syscall_nr_unlinkat          = 290;
      syscall_nr_renameat          = 291;
      syscall_nr_linkat            = 292;
      syscall_nr_symlinkat         = 293;
      syscall_nr_readlinkat        = 294;
      syscall_nr_fchmodat          = 295;
      syscall_nr_faccessat         = 296;
      syscall_nr_pselect6          = 297;
      syscall_nr_ppoll             = 298;
      syscall_nr_unshare           = 299;
      syscall_nr_set_robust_list   = 300;
      syscall_nr_get_robust_list   = 301;
      syscall_nr_migrate_pages     = 302;
      syscall_nr_mbind             = 303;
      syscall_nr_get_mempolicy     = 304;
      syscall_nr_set_mempolicy     = 305;
      syscall_nr_kexec_load        = 306;
      syscall_nr_move_pages        = 307;
      syscall_nr_getcpu            = 308;
      syscall_nr_epoll_pwait       = 309;
      syscall_nr_utimensat         = 310;
      syscall_nr_signalfd          = 311;
      syscall_nr_timerfd_create    = 312;
      syscall_nr_eventfd           = 313;
      syscall_nr_fallocate         = 314;
      syscall_nr_timerfd_settime   = 315;
      syscall_nr_timerfd_gettime   = 316;
      syscall_nr_signalfd4         = 317;
      syscall_nr_eventfd2          = 318;
      syscall_nr_epoll_create1     = 319;
      syscall_nr_dup3              = 320;
      syscall_nr_pipe2             = 321;
      syscall_nr_inotify_init1     = 322;
      syscall_nr_accept4           = 323;
      syscall_nr_preadv            = 324;
      syscall_nr_pwritev           = 325;
      syscall_nr_rt_tgsigqueueinfo = 326;
      syscall_nr_perf_event_open   = 327;
      syscall_nr_recvmmsg          = 328;
      syscall_nr_fanotify_init     = 329;
      syscall_nr_fanotify_mark     = 330;
      syscall_nr_prlimit64         = 331;
      syscall_nr_name_to_handle_at = 332;
      syscall_nr_open_by_handle_at = 333;
      syscall_nr_clock_adjtime     = 334;
      syscall_nr_syncfs            = 335;
      syscall_nr_sendmmsg          = 336;
      syscall_nr_setns             = 337;
      syscall_nr_process_vm_readv  = 338;
      syscall_nr_process_vm_writev = 339;
      syscall_nr_kern_features     = 340;
      syscall_nr_kcmp              = 341;
      syscall_nr_finit_module      = 342;
      syscall_nr_sched_setattr     = 343;
      syscall_nr_sched_getattr     = 344;
      syscall_nr_renameat2         = 345;
      syscall_nr_seccomp           = 346;
      syscall_nr_getrandom         = 347;
      syscall_nr_memfd_create      = 348;
      syscall_nr_bpf               = 349;
      syscall_nr_execveat          = 350;
      syscall_nr_membarrier        = 351;
      syscall_nr_userfaultfd       = 352;
      syscall_nr_bind              = 353;
      syscall_nr_listen            = 354;
      syscall_nr_setsockopt        = 355;
      syscall_nr_mlock2            = 356;
      syscall_nr_copy_file_range   = 357;
      syscall_nr_preadv2           = 358;
      syscall_nr_pwritev2          = 359;
      syscall_nr_statx             = 360;
      syscall_nr_io_pgetevents     = 361;
      syscall_nr_pkey_mprotect                = 362;
      syscall_nr_pkey_alloc                   = 363;
      syscall_nr_pkey_free                    = 364;
      syscall_nr_rseq                         = 365;
      syscall_nr_semtimedop                   = 392;
      syscall_nr_semget                       = 393;
      syscall_nr_semctl                       = 394;
      syscall_nr_shmget                       = 395;
      syscall_nr_shmctl                       = 396;
      syscall_nr_shmat                        = 397;
      syscall_nr_shmdt                        = 398;
      syscall_nr_msgget                       = 399;
      syscall_nr_msgsnd                       = 400;
      syscall_nr_msgrcv                       = 401;
      syscall_nr_msgctl                       = 402;
      syscall_nr_clock_gettime64              = 403;
      syscall_nr_clock_settime64              = 404;
      syscall_nr_clock_adjtime64              = 405;
      syscall_nr_clock_getres_time64          = 406;
      syscall_nr_clock_nanosleep_time64       = 407;
      syscall_nr_timer_gettime64              = 408;
      syscall_nr_timer_settime64              = 409;
      syscall_nr_timerfd_gettime64            = 410;
      syscall_nr_timerfd_settime64            = 411;
      syscall_nr_utimensat_time64             = 412;
      syscall_nr_pselect6_time64              = 413;
      syscall_nr_ppoll_time64                 = 414;
      syscall_nr_io_pgetevents_time64         = 416;
      syscall_nr_recvmmsg_time64              = 417;
      syscall_nr_mq_timedsend_time64          = 418;
      syscall_nr_mq_timedreceive_time64       = 419;
      syscall_nr_semtimedop_time64            = 420;
      syscall_nr_rt_sigtimedwait_time64       = 421;
      syscall_nr_futex_time64                 = 422;
      syscall_nr_sched_rr_get_interval_time64 = 423;
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

