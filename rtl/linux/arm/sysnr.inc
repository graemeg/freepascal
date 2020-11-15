{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003-2004 by Florian Klaempfl,
    member of the Free Pascal development team.

    Syscall nrs for arm-linux

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
{$ifdef FPC_ABI_EABI}
  syscall_nr_base = $0;
{$else FPC_ABI_EABI}
   syscall_nr_base = $900000;
{$endif FPC_ABI_EABI}

  syscall_nr_exit                       =  syscall_nr_base+ 1;
  syscall_nr_fork                       =  syscall_nr_base+ 2;
  syscall_nr_read                       =  syscall_nr_base+ 3;
  syscall_nr_write                      =  syscall_nr_base+ 4;
  syscall_nr_open                       =  syscall_nr_base+ 5;
  syscall_nr_close                      =  syscall_nr_base+ 6;
  // not supported: syscall_nr_waitpid                    =  syscall_nr_base+ 7;
  syscall_nr_creat                      =  syscall_nr_base+ 8;
  syscall_nr_link                       =  syscall_nr_base+ 9;
  syscall_nr_unlink                     =  syscall_nr_base+10;
  syscall_nr_execve                     =  syscall_nr_base+11;
  syscall_nr_chdir                      =  syscall_nr_base+12;
{$ifndef FPC_ABI_EABI}
  syscall_nr_time                       =  syscall_nr_base+13;
{$endif}
  syscall_nr_mknod                      =  syscall_nr_base+14;
  syscall_nr_chmod                      =  syscall_nr_base+15;
  syscall_nr_lchown                     =  syscall_nr_base+16;
  syscall_nr_break                      =  syscall_nr_base+17;

  syscall_nr_lseek                      =  syscall_nr_base+19;
  syscall_nr_getpid                     =  syscall_nr_base+20;
  syscall_nr_mount                      =  syscall_nr_base+21;
{$ifndef FPC_ABI_EABI}
  syscall_nr_umount                     =  syscall_nr_base+22;
{$endif}
  syscall_nr_setuid                     =  syscall_nr_base+23;
  syscall_nr_getuid                     =  syscall_nr_base+24;
{$ifndef FPC_ABI_EABI}
  syscall_nr_stime                      =  syscall_nr_base+25;
{$endif}
  syscall_nr_ptrace                     =  syscall_nr_base+26;
{$ifndef FPC_ABI_EABI}
  syscall_nr_alarm                      =  syscall_nr_base+27;
{$endif}

  syscall_nr_pause                      =  syscall_nr_base+29;
{$ifndef FPC_ABI_EABI}
  syscall_nr_utime                      =  syscall_nr_base+30;
{$endif}
  syscall_nr_stty                       =  syscall_nr_base+31;
  syscall_nr_gtty                       =  syscall_nr_base+32;
  syscall_nr_access                     =  syscall_nr_base+33;
  syscall_nr_nice                       =  syscall_nr_base+34;
  syscall_nr_ftime                      =  syscall_nr_base+35;
  syscall_nr_sync                       =  syscall_nr_base+36;
  syscall_nr_kill                       =  syscall_nr_base+37;
  syscall_nr_rename                     =  syscall_nr_base+38;
  syscall_nr_mkdir                      =  syscall_nr_base+39;
  syscall_nr_rmdir                      =  syscall_nr_base+40;
  syscall_nr_dup                        =  syscall_nr_base+41;
  syscall_nr_pipe                       =  syscall_nr_base+42;
  syscall_nr_times                      =  syscall_nr_base+43;
  syscall_nr_prof                       =  syscall_nr_base+44;
  syscall_nr_brk                        =  syscall_nr_base+45;
  syscall_nr_setgid                     =  syscall_nr_base+46;
  syscall_nr_getgid                     =  syscall_nr_base+47;
  syscall_nr_signal                     =  syscall_nr_base+48;
  syscall_nr_geteuid                    =  syscall_nr_base+49;
  syscall_nr_getegid                    =  syscall_nr_base+50;
  syscall_nr_acct                       =  syscall_nr_base+51;
  syscall_nr_umount2                    =  syscall_nr_base+52;
  syscall_nr_lock                       =  syscall_nr_base+53;
  syscall_nr_ioctl                      =  syscall_nr_base+54;
  syscall_nr_fcntl                      =  syscall_nr_base+55;
  syscall_nr_mpx                        =  syscall_nr_base+56;
  syscall_nr_setpgid                    =  syscall_nr_base+57;
  syscall_nr_ulimit                     =  syscall_nr_base+58;

  syscall_nr_umask                      =  syscall_nr_base+60;
  syscall_nr_chroot                     =  syscall_nr_base+61;
  syscall_nr_ustat                      =  syscall_nr_base+62;
  syscall_nr_dup2                       =  syscall_nr_base+63;
  syscall_nr_getppid                    =  syscall_nr_base+64;
  syscall_nr_getpgrp                    =  syscall_nr_base+65;
  syscall_nr_setsid                     =  syscall_nr_base+66;
  syscall_nr_sigaction                  =  syscall_nr_base+67;
  syscall_nr_sgetmask                   =  syscall_nr_base+68;
  syscall_nr_ssetmask                   =  syscall_nr_base+69;
  syscall_nr_setreuid                   =  syscall_nr_base+70;
  syscall_nr_setregid                   =  syscall_nr_base+71;
  syscall_nr_sigsuspend                 =  syscall_nr_base+72;
  syscall_nr_sigpending                 =  syscall_nr_base+73;
  syscall_nr_sethostname                =  syscall_nr_base+74;
  syscall_nr_setrlimit                  =  syscall_nr_base+75;
{$ifndef FPC_ABI_EABI}
  syscall_nr_getrlimit                  =  syscall_nr_base+76;
{$endif}
  syscall_nr_getrusage                  =  syscall_nr_base+77;
  syscall_nr_gettimeofday               =  syscall_nr_base+78;
  syscall_nr_settimeofday               =  syscall_nr_base+79;
  syscall_nr_getgroups                  =  syscall_nr_base+80;
  syscall_nr_setgroups                  =  syscall_nr_base+81;
{$ifndef FPC_ABI_EABI}
  syscall_nr_select                     =  syscall_nr_base+82;
{$endif}
  syscall_nr_symlink                    =  syscall_nr_base+83;

  syscall_nr_readlink                   =  syscall_nr_base+85;
  syscall_nr_uselib                     =  syscall_nr_base+86;
  syscall_nr_swapon                     =  syscall_nr_base+87;
  syscall_nr_reboot                     =  syscall_nr_base+88;
{$ifndef FPC_ABI_EABI}
  syscall_nr_readdir                    =  syscall_nr_base+89;
  syscall_nr_mmap                       =  syscall_nr_base+90;
{$endif}
  syscall_nr_munmap                     =  syscall_nr_base+91;
  syscall_nr_truncate                   =  syscall_nr_base+92;
  syscall_nr_ftruncate                  =  syscall_nr_base+93;
  syscall_nr_fchmod                     =  syscall_nr_base+94;
  syscall_nr_fchown                     =  syscall_nr_base+95;
  syscall_nr_getpriority                =  syscall_nr_base+96;
  syscall_nr_setpriority                =  syscall_nr_base+97;
  syscall_nr_profil                     =  syscall_nr_base+98;
  syscall_nr_statfs                     =  syscall_nr_base+99;
  syscall_nr_fstatfs                    = syscall_nr_base+100;
  syscall_nr_ioperm                     = syscall_nr_base+101;
{$ifndef FPC_ABI_EABI}
  syscall_nr_socketcall                 = syscall_nr_base+102;
{$endif}
  syscall_nr_syslog                     = syscall_nr_base+103;
  syscall_nr_setitimer                  = syscall_nr_base+104;
  syscall_nr_getitimer                  = syscall_nr_base+105;
  syscall_nr_stat                       = syscall_nr_base+106;
  syscall_nr_lstat                      = syscall_nr_base+107;
  syscall_nr_fstat                      = syscall_nr_base+108;
  syscall_nr_vhangup                    = syscall_nr_base+111;
  syscall_nr_idle                       = syscall_nr_base+112;
{$ifndef FPC_ABI_EABI}
  syscall_nr_syscall                    = syscall_nr_base+113;
{$endif}
  syscall_nr_wait4                      = syscall_nr_base+114;
  syscall_nr_swapoff                    = syscall_nr_base+115;
  syscall_nr_sysinfo                    = syscall_nr_base+116;
{$ifndef FPC_ABI_EABI}
  syscall_nr_ipc                        = syscall_nr_base+117;
{$endif}
  syscall_nr_fsync                      = syscall_nr_base+118;
  syscall_nr_sigreturn                  = syscall_nr_base+119;
  syscall_nr_clone                      = syscall_nr_base+120;
  syscall_nr_setdomainname              = syscall_nr_base+121;
  syscall_nr_uname                      = syscall_nr_base+122;
  syscall_nr_modify_ldt                 = syscall_nr_base+123;
  syscall_nr_adjtimex                   = syscall_nr_base+124;
  syscall_nr_mprotect                   = syscall_nr_base+125;
  syscall_nr_sigprocmask                = syscall_nr_base+126;
  syscall_nr_create_module              = syscall_nr_base+127;
  syscall_nr_init_module                = syscall_nr_base+128;
  syscall_nr_delete_module              = syscall_nr_base+129;
  syscall_nr_get_kernel_syms            = syscall_nr_base+130;
  syscall_nr_quotactl                   = syscall_nr_base+131;
  syscall_nr_getpgid                    = syscall_nr_base+132;
  syscall_nr_fchdir                     = syscall_nr_base+133;
  syscall_nr_bdflush                    = syscall_nr_base+134;
  syscall_nr_sysfs                      = syscall_nr_base+135;
  syscall_nr_personality                = syscall_nr_base+136;
  syscall_nr_afs_syscall                = syscall_nr_base+137;
  syscall_nr_setfsuid                   = syscall_nr_base+138;
  syscall_nr_setfsgid                   = syscall_nr_base+139;
  syscall_nr__llseek                    = syscall_nr_base+140;
  syscall_nr_getdents                   = syscall_nr_base+141;
  syscall_nr__newselect                 = syscall_nr_base+142;
  syscall_nr_flock                      = syscall_nr_base+143;
  syscall_nr_msync                      = syscall_nr_base+144;
  syscall_nr_readv                      = syscall_nr_base+145;
  syscall_nr_writev                     = syscall_nr_base+146;
  syscall_nr_getsid                     = syscall_nr_base+147;
  syscall_nr_fdatasync                  = syscall_nr_base+148;
  syscall_nr__sysctl                    = syscall_nr_base+149;
  syscall_nr_mlock                      = syscall_nr_base+150;
  syscall_nr_munlock                    = syscall_nr_base+151;
  syscall_nr_mlockall                   = syscall_nr_base+152;
  syscall_nr_munlockall                 = syscall_nr_base+153;
  syscall_nr_sched_setparam             = syscall_nr_base+154;
  syscall_nr_sched_getparam             = syscall_nr_base+155;
  syscall_nr_sched_setscheduler         = syscall_nr_base+156;
  syscall_nr_sched_getscheduler         = syscall_nr_base+157;
  syscall_nr_sched_yield                = syscall_nr_base+158;
  syscall_nr_sched_get_priority_max     = syscall_nr_base+159;
  syscall_nr_sched_get_priority_min     = syscall_nr_base+160;
  syscall_nr_sched_rr_get_interval      = syscall_nr_base+161;
  syscall_nr_nanosleep                  = syscall_nr_base+162;
  syscall_nr_mremap                     = syscall_nr_base+163;
  syscall_nr_setresuid                  = syscall_nr_base+164;
  syscall_nr_getresuid                  = syscall_nr_base+165;
  syscall_nr_vm86                       = syscall_nr_base+166;
  syscall_nr_query_module               = syscall_nr_base+167;
  syscall_nr_poll                       = syscall_nr_base+168;
  syscall_nr_nfsservctl                 = syscall_nr_base+169;
  syscall_nr_setresgid                  = syscall_nr_base+170;
  syscall_nr_getresgid                  = syscall_nr_base+171;
  syscall_nr_prctl                      = syscall_nr_base+172;
  syscall_nr_rt_sigreturn               = syscall_nr_base+173;
  syscall_nr_rt_sigaction               = syscall_nr_base+174;
  syscall_nr_rt_sigprocmask             = syscall_nr_base+175;
  syscall_nr_rt_sigpending              = syscall_nr_base+176;
  syscall_nr_rt_sigtimedwait            = syscall_nr_base+177;
  syscall_nr_rt_sigqueueinfo            = syscall_nr_base+178;
  syscall_nr_rt_sigsuspend              = syscall_nr_base+179;
  syscall_nr_pread64                    = syscall_nr_base+180;
  syscall_nr_pwrite64                   = syscall_nr_base+181;
  syscall_nr_chown                      = syscall_nr_base+182;
  syscall_nr_getcwd                     = syscall_nr_base+183;
  syscall_nr_capget                     = syscall_nr_base+184;
  syscall_nr_capset                     = syscall_nr_base+185;
  syscall_nr_sigaltstack                = syscall_nr_base+186;
  syscall_nr_sendfile                   = syscall_nr_base+187;
  syscall_nr_vfork                      = syscall_nr_base+190;
  syscall_nr_ugetrlimit                 = syscall_nr_base+191;
  syscall_nr_mmap2                      = syscall_nr_base+192;
  syscall_nr_truncate64                 = syscall_nr_base+193;
  syscall_nr_ftruncate64                = syscall_nr_base+194;
  syscall_nr_stat64                     = syscall_nr_base+195;
  syscall_nr_lstat64                    = syscall_nr_base+196;
  syscall_nr_fstat64                    = syscall_nr_base+197;
  syscall_nr_lchown32                   = syscall_nr_base+198;
  syscall_nr_getuid32                   = syscall_nr_base+199;
  syscall_nr_getgid32                   = syscall_nr_base+200;
  syscall_nr_geteuid32                  = syscall_nr_base+201;
  syscall_nr_getegid32                  = syscall_nr_base+202;
  syscall_nr_setreuid32                 = syscall_nr_base+203;
  syscall_nr_setregid32                 = syscall_nr_base+204;
  syscall_nr_getgroups32                = syscall_nr_base+205;
  syscall_nr_setgroups32                = syscall_nr_base+206;
  syscall_nr_fchown32                   = syscall_nr_base+207;
  syscall_nr_setresuid32                = syscall_nr_base+208;
  syscall_nr_getresuid32                = syscall_nr_base+209;
  syscall_nr_setresgid32                = syscall_nr_base+210;
  syscall_nr_getresgid32                = syscall_nr_base+211;
  syscall_nr_chown32                    = syscall_nr_base+212;
  syscall_nr_setuid32                   = syscall_nr_base+213;
  syscall_nr_setgid32                   = syscall_nr_base+214;
  syscall_nr_setfsuid32                 = syscall_nr_base+215;
  syscall_nr_setfsgid32                 = syscall_nr_base+216;
  syscall_nr_getdents64                 = syscall_nr_base+217;
  syscall_nr_pivot_root                 = syscall_nr_base+218;
  syscall_nr_mincore                    = syscall_nr_base+219;
  syscall_nr_madvise                    = syscall_nr_base+220;
  syscall_nr_fcntl64                    = syscall_nr_base+221;
  syscall_nr_security                   = syscall_nr_base+223;
  syscall_nr_gettid                     = syscall_nr_base+224;
  syscall_nr_readahead                  = syscall_nr_base+225;
  syscall_nr_setxattr                   = syscall_nr_base+226;
  syscall_nr_lsetxattr                  = syscall_nr_base+227;
  syscall_nr_fsetxattr                  = syscall_nr_base+228;
  syscall_nr_getxattr                   = syscall_nr_base+229;
  syscall_nr_lgetxattr                  = syscall_nr_base+230;
  syscall_nr_fgetxattr                  = syscall_nr_base+231;
  syscall_nr_listxattr                  = syscall_nr_base+232;
  syscall_nr_llistxattr                 = syscall_nr_base+233;
  syscall_nr_flistxattr                 = syscall_nr_base+234;
  syscall_nr_removexattr                = syscall_nr_base+235;
  syscall_nr_lremovexattr               = syscall_nr_base+236;
  syscall_nr_fremovexattr               = syscall_nr_base+237;
  syscall_nr_tkill                      = syscall_nr_base+238;
  syscall_nr_sendfile64                 = syscall_nr_base+239;
  syscall_nr_futex                      = syscall_nr_base+240;
  syscall_nr_sched_setaffinity          = syscall_nr_base+241;
  syscall_nr_sched_getaffinity          = syscall_nr_base+242;
  syscall_nr_io_setup                   = syscall_nr_base+243;
  syscall_nr_io_destroy                 = syscall_nr_base+244;
  syscall_nr_io_getevents               = syscall_nr_base+245;
  syscall_nr_io_submit                  = syscall_nr_base+246;
  syscall_nr_io_cancel                  = syscall_nr_base+247;
  syscall_nr_exit_group                 = syscall_nr_base+248;
  syscall_nr_lookup_dcookie             = syscall_nr_base+249;
  syscall_nr_epoll_create               = syscall_nr_base+250;
  syscall_nr_epoll_ctl                  = syscall_nr_base+251;
  syscall_nr_epoll_wait                 = syscall_nr_base+252;
  syscall_nr_remap_file_pages           = syscall_nr_base+253;
  syscall_nr_set_tid_address            = syscall_nr_base+256;
                                        { 254 for set_thread_area }
                                        { 255 for get_thread_area }
  syscall_nr_timer_create               = syscall_nr_base+257;
  syscall_nr_timer_settime              = syscall_nr_base+258;
  syscall_nr_timer_gettime              = syscall_nr_base+259;
  syscall_nr_timer_getoverrun           = syscall_nr_base+260;
  syscall_nr_timer_delete               = syscall_nr_base+261;
  syscall_nr_clock_settime              = syscall_nr_base+262;
  syscall_nr_clock_gettime              = syscall_nr_base+263;
  syscall_nr_clock_getres               = syscall_nr_base+264;
  syscall_nr_clock_nanosleep            = syscall_nr_base+265;
  syscall_nr_statfs64                   = syscall_nr_base+266;
  syscall_nr_fstatfs64                  = syscall_nr_base+267;
  syscall_nr_tgkill                     = syscall_nr_base+268;
  syscall_nr_utimes                     = syscall_nr_base+269;
  syscall_nr_fadvise64_64               = syscall_nr_base+270;
  syscall_nr_pciconfig_iobase           = syscall_nr_base+271;
  syscall_nr_pciconfig_read             = syscall_nr_base+272;
  syscall_nr_pciconfig_write            = syscall_nr_base+273;
  syscall_nr_mq_open                    = syscall_nr_base+274;
  syscall_nr_mq_unlink                  = syscall_nr_base+275;
  syscall_nr_mq_timedsend               = syscall_nr_base+276;
  syscall_nr_mq_timedreceive            = syscall_nr_base+277;
  syscall_nr_mq_notify                  = syscall_nr_base+278;
  syscall_nr_mq_getsetattr              = syscall_nr_base+279;
  syscall_nr_waitid                     = syscall_nr_base+280;
  syscall_nr_socket                     = syscall_nr_base+281;
  syscall_nr_bind                       = syscall_nr_base+282;
  syscall_nr_connect                    = syscall_nr_base+283;
  syscall_nr_listen                     = syscall_nr_base+284;
  syscall_nr_accept                     = syscall_nr_base+285;
  syscall_nr_getsockname                = syscall_nr_base+286;
  syscall_nr_getpeername                = syscall_nr_base+287;
  syscall_nr_socketpair                 = syscall_nr_base+288;
  syscall_nr_send                       = syscall_nr_base+289;
  syscall_nr_sendto                     = syscall_nr_base+290;
  syscall_nr_recv                       = syscall_nr_base+291;
  syscall_nr_recvfrom                   = syscall_nr_base+292;
  syscall_nr_shutdown                   = syscall_nr_base+293;
  syscall_nr_setsockopt                 = syscall_nr_base+294;
  syscall_nr_getsockopt                 = syscall_nr_base+295;
  syscall_nr_sendmsg                    = syscall_nr_base+296;
  syscall_nr_recvmsg                    = syscall_nr_base+297;
  syscall_nr_semop                      = syscall_nr_base+298;
  syscall_nr_semget                     = syscall_nr_base+299;
  syscall_nr_semctl                     = syscall_nr_base+300;
  syscall_nr_msgsnd                     = syscall_nr_base+301;
  syscall_nr_msgrcv                     = syscall_nr_base+302;
  syscall_nr_msgget                     = syscall_nr_base+303;
  syscall_nr_msgctl                     = syscall_nr_base+304;
  syscall_nr_shmat                      = syscall_nr_base+305;
  syscall_nr_shmdt                      = syscall_nr_base+306;
  syscall_nr_shmget                     = syscall_nr_base+307;
  syscall_nr_shmctl                     = syscall_nr_base+308;
  syscall_nr_add_key                    = syscall_nr_base+309;
  syscall_nr_request_key                = syscall_nr_base+310;
  syscall_nr_keyctl                     = syscall_nr_base+311;
  syscall_nr_semtimedop                 = syscall_nr_base+312;
  syscall_nr_vserver                    = syscall_nr_base+313;
  syscall_nr_ioprio_set                 = syscall_nr_base+314;
  syscall_nr_ioprio_get                 = syscall_nr_base+315;
  syscall_nr_inotify_init               = syscall_nr_base+316;
  syscall_nr_inotify_add_watch          = syscall_nr_base+317;
  syscall_nr_inotify_rm_watch           = syscall_nr_base+318;
  syscall_nr_mbind                      = syscall_nr_base+319;
  syscall_nr_get_mempolicy              = syscall_nr_base+320;
  syscall_nr_set_mempolicy              = syscall_nr_base+321;
  syscall_nr_openat                     = syscall_nr_base+322;
  syscall_nr_mkdirat                    = syscall_nr_base+323;
  syscall_nr_mknodat                    = syscall_nr_base+324;
  syscall_nr_fchownat                   = syscall_nr_base+325;
  syscall_nr_futimesat                  = syscall_nr_base+326;
  syscall_nr_fstatat64                  = syscall_nr_base+327;
  syscall_nr_unlinkat                   = syscall_nr_base+328;
  syscall_nr_renameat                   = syscall_nr_base+329;
  syscall_nr_linkat                     = syscall_nr_base+330;
  syscall_nr_symlinkat                  = syscall_nr_base+331;
  syscall_nr_readlinkat                 = syscall_nr_base+332;
  syscall_nr_fchmodat                   = syscall_nr_base+333;
  syscall_nr_faccessat                  = syscall_nr_base+334;
                                        {* 335 for pselect6 *}
                                        {* 336 for ppoll *}
  syscall_nr_unshare                    = syscall_nr_base+337;
  syscall_nr_set_robust_list            = syscall_nr_base+338;
  syscall_nr_get_robust_list            = syscall_nr_base+339;
  syscall_nr_splice                     = syscall_nr_base+340;
  syscall_nr_arm_sync_file_range        = syscall_nr_base+341;
  syscall_nr_sync_file_range2           = syscall_nr_arm_sync_file_range;
  syscall_nr_tee                        = syscall_nr_base+342;
  syscall_nr_vmsplice                   = syscall_nr_base+343;
  syscall_nr_move_pages                 = syscall_nr_base+344;
  syscall_nr_getcpu                     = syscall_nr_base+345;
  syscall_nr_epoll_pwait                = syscall_nr_base+346;
  syscall_nr_kexec_load                 = syscall_nr_base+347;
  syscall_nr_utimensat                  = syscall_nr_base+348;
  syscall_nr_signalfd                   = syscall_nr_base+349;
  syscall_nr_timerfd_create             = syscall_nr_base+350;
  syscall_nr_eventfd                    = syscall_nr_base+351;
  syscall_nr_fallocate                  = syscall_nr_base+352;
  syscall_nr_timerfd_settime            = syscall_nr_base+353;
  syscall_nr_timerfd_gettime            = syscall_nr_base+354;
  syscall_nr_signalfd4                  = syscall_nr_base+355;
  syscall_nr_eventfd2                   = syscall_nr_base+356;
  syscall_nr_epoll_create1              = syscall_nr_base+357;
  syscall_nr_dup3                       = syscall_nr_base+358;
  syscall_nr_pipe2                      = syscall_nr_base+359;
  syscall_nr_inotify_init1              = syscall_nr_base+360;
  syscall_nr_preadv                     = syscall_nr_base+361;
  syscall_nr_pwritev                    = syscall_nr_base+362;
  syscall_nr_rt_tgsigqueueinfo          = syscall_nr_base+363;
  syscall_nr_perf_event_open            = syscall_nr_base+364;
  syscall_nr_recvmmsg                   = syscall_nr_base+365;
  syscall_nr_accept4                      = syscall_nr_base+366;
  syscall_nr_fanotify_init                = syscall_nr_base+367;
  syscall_nr_fanotify_mark                = syscall_nr_base+368;
  syscall_nr_prlimit64                    = syscall_nr_base+369;
  syscall_nr_name_to_handle_at            = syscall_nr_base+370;
  syscall_nr_open_by_handle_at            = syscall_nr_base+371;
  syscall_nr_clock_adjtime                = syscall_nr_base+372;
  syscall_nr_syncfs                       = syscall_nr_base+373;
  syscall_nr_sendmmsg                     = syscall_nr_base+374;
  syscall_nr_setns                        = syscall_nr_base+375;
  syscall_nr_process_vm_readv             = syscall_nr_base+376;
  syscall_nr_process_vm_writev            = syscall_nr_base+377;
  syscall_nr_kcmp                         = syscall_nr_base+378;
  syscall_nr_finit_module                 = syscall_nr_base+379;
  syscall_nr_sched_setattr                = syscall_nr_base+380;
  syscall_nr_sched_getattr                = syscall_nr_base+381;
  syscall_nr_renameat2                    = syscall_nr_base+382;
  syscall_nr_seccomp                      = syscall_nr_base+383;
  syscall_nr_getrandom                    = syscall_nr_base+384;
  syscall_nr_memfd_create                 = syscall_nr_base+385;
  syscall_nr_bpf                          = syscall_nr_base+386;
  syscall_nr_execveat                     = syscall_nr_base+387;
  syscall_nr_userfaultfd                  = syscall_nr_base+388;
  syscall_nr_membarrier                   = syscall_nr_base+389;
  syscall_nr_mlock2                       = syscall_nr_base+390;
  syscall_nr_copy_file_range              = syscall_nr_base+391;
  syscall_nr_preadv2                      = syscall_nr_base+392;
  syscall_nr_pwritev2                     = syscall_nr_base+393;
  syscall_nr_pkey_mprotect                = syscall_nr_base+394;
  syscall_nr_pkey_alloc                   = syscall_nr_base+395;
  syscall_nr_pkey_free                    = syscall_nr_base+396;
  syscall_nr_statx                        = syscall_nr_base+397;
  syscall_nr_rseq                         = syscall_nr_base+398;
  syscall_nr_io_pgetevents                = syscall_nr_base+399;
  syscall_nr_migrate_pages                = syscall_nr_base+400;
  syscall_nr_kexec_file_load              = syscall_nr_base+401;
  syscall_nr_clock_gettime64              = syscall_nr_base+403;
  syscall_nr_clock_settime64              = syscall_nr_base+404;
  syscall_nr_clock_adjtime64              = syscall_nr_base+405;
  syscall_nr_clock_getres_time64          = syscall_nr_base+406;
  syscall_nr_clock_nanosleep_time64       = syscall_nr_base+407;
  syscall_nr_timer_gettime64              = syscall_nr_base+408;
  syscall_nr_timer_settime64              = syscall_nr_base+409;
  syscall_nr_timerfd_gettime64            = syscall_nr_base+410;
  syscall_nr_timerfd_settime64            = syscall_nr_base+411;
  syscall_nr_utimensat_time64             = syscall_nr_base+412;
  syscall_nr_pselect6_time64              = syscall_nr_base+413;
  syscall_nr_ppoll_time64                 = syscall_nr_base+414;
  syscall_nr_io_pgetevents_time64         = syscall_nr_base+416;
  syscall_nr_recvmmsg_time64              = syscall_nr_base+417;
  syscall_nr_mq_timedsend_time64          = syscall_nr_base+418;
  syscall_nr_mq_timedreceive_time64       = syscall_nr_base+419;
  syscall_nr_semtimedop_time64            = syscall_nr_base+420;
  syscall_nr_rt_sigtimedwait_time64       = syscall_nr_base+421;
  syscall_nr_futex_time64                 = syscall_nr_base+422;
  syscall_nr_sched_rr_get_interval_time64 = syscall_nr_base+423;
  syscall_nr_pidfd_send_signal            = syscall_nr_base+424;
  syscall_nr_io_uring_setup               = syscall_nr_base+425;
  syscall_nr_io_uring_enter               = syscall_nr_base+426;
  syscall_nr_io_uring_register            = syscall_nr_base+427;
  syscall_nr_open_tree                    = syscall_nr_base+428;
  syscall_nr_move_mount                   = syscall_nr_base+429;
  syscall_nr_fsopen                       = syscall_nr_base+430;
  syscall_nr_fsconfig                     = syscall_nr_base+431;
  syscall_nr_fsmount                      = syscall_nr_base+432;
  syscall_nr_fspick                       = syscall_nr_base+433;
  syscall_nr_pidfd_open                   = syscall_nr_base+434;
  syscall_nr_clone3                       = syscall_nr_base+435;

{*
 * The following SWIs are ARM private.
 *}
  __ARM_NR_BASE                         = syscall_nr_base+$f0000;
  syscall_nr___ARM_NR_breakpoint        = __ARM_NR_BASE+1;
  syscall_nr___ARM_NR_cacheflush        = __ARM_NR_BASE+2;
  syscall_nr___ARM_NR_usr26             = __ARM_NR_BASE+3;
  syscall_nr___ARM_NR_usr32             = __ARM_NR_BASE+4;
  syscall_nr___ARM_NR_set_tls           = __ARM_NR_BASE+5;
