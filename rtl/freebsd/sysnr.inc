{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
{More or less checked/in use FreeBSD syscalls}
 syscall_nr_readv                       = 120;
 syscall_nr_writev                      = 121;
 syscall_nr_pread                       = 173;
 syscall_nr_pwrite                      = 174;
 syscall_nr_semsys                      = 169;
 syscall_nr_msgsys                      = 170;
 syscall_nr_shmsys                      = 171;
 syscall_nr_mkfifo                      = 132;
 syscall_nr___getcwd                    = 326;
 syscall_nr_settimeofday                = 122;
 syscall_nr_getitimer                   =  86;
 syscall_nr_setitimer                   =  83;
 syscall_nr___syscall                   = 198;
 syscall_nr_setsid                      = 147;
 syscall_nr_getpgrp                     =  81;
 syscall_nr_setuid                      =  23;
 syscall_nr_setgid                      = 181;
 syscall_nr_getgroups                   =  79;
 syscall_nr_sysarch                     = 165;
 syscall_nr_accept                      =  30;
 syscall_nr_access                      =  33;
 syscall_nr_bind                        = 104;
 syscall_nr_chdir                       =  12;
 syscall_nr_chmod                       =  15;
 syscall_nr_chown                       =  16;
 syscall_nr_close                       =   6;
 syscall_nr_connect                     =  98;
 syscall_nr_dup                         =  41;
 syscall_nr_dup2                        =  90;
 syscall_nr_execve                      =  59;
 syscall_nr_exit                        =   1;
 syscall_nr_fcntl                       =  92;
 syscall_nr_flock                       = 131;
 syscall_nr_fork                        =   2;
 syscall_nr_fstat                       = 189;
 syscall_nr_statfs4                     = 157;  // COMPAT4x
 syscall_nr_fstatfs4                    = 158;  // COMPAT4x
 syscall_nr_getfsstat                   = 395;  
 syscall_nr_statfs                      = 396;  
 syscall_nr_fstatfs                     = 397;  
 syscall_nr_fsync                       =  95;
 syscall_nr_ftruncate                   = 201;
 syscall_nr_getdents                    = 272;
 syscall_nr_getegid                     =  43;
 syscall_nr_geteuid                     =  25;
 syscall_nr_getgid                      =  47;
 syscall_nr_getpeername                 =  31;
 syscall_nr_getpid                      =  20;
 syscall_nr_getppid                     =  39;
 syscall_nr_getpriority                 = 100;
 syscall_nr_getsockname                 =  32;
 syscall_nr_gettimeofday                = 116;
 syscall_nr_getuid                      =  24;
 syscall_nr_ioctl                       =  54;
 syscall_nr_kill                        =  37;
 syscall_nr_link                        =   9;
 syscall_nr_listen                      = 106;
 syscall_nr_lseek                       = 199;
 syscall_nr_lstat                       = 190;
 syscall_nr_mkdir                       = 136;
 syscall_nr_mknod                       =  14;
 syscall_nr_mmap                        = 197;
 syscall_nr_open                        =   5;
 syscall_nr_pipe                        =  42;
 syscall_nr_poll                        = 209;
 syscall_nr_read                        =   3;
 syscall_nr_readlink                    =  58;
 syscall_nr_recvfrom                    =  29;
 syscall_nr_rename                      = 128;
 syscall_nr_rmdir                       = 137;
 syscall_nr_select                      =  93;
 syscall_nr_sendto                      = 133;
 syscall_nr_setpriority                 =  96;
 syscall_nr_setsockopt                  = 105;
 syscall_nr_shutdown                    = 134;
 syscall_nr_sigaction                   = 416;
 syscall_nr_sigpending                  = 343;
 syscall_nr_sigprocmask                 = 340;
 syscall_nr_sigsuspend                  = 341;
 syscall_nr_sigtimedwait                = 345;
 syscall_nr_socket                      =  97;
 syscall_nr_socketpair                  = 135;
 syscall_nr_stat                        = 188;
 syscall_nr_symlink                     =  57;
 syscall_nr_umask                       =  60;
 syscall_nr_unlink                      =  10;
 syscall_nr_utimes                      = 138;
 syscall_nr_waitpid                     =   7;
 syscall_nr_write                       =   4;
 syscall_nr_munmap                      =  73;
 syscall_nr_getsockopt                  = 118;
 syscall_nr_rfork                       = 251;
 syscall_nr_clock_gettime               = 232;
 syscall_nr_clock_settime               = 233;
 syscall_nr_clock_getres                = 234;
 syscall_nr_nanosleep                   = 240;
 syscall_nr_getrusage                   = 117;
 syscall_nr_uuidgen			= 392; { 5.x+}
 syscall_nr_kqueue 			= 362;
 syscall_nr_kevent 			= 363;
 syscall_nr_kldload                     = 304;
 syscall_nr_kldunload                   = 305;
 syscall_nr_kldfind                     = 306;
 syscall_nr_kldnext                     = 307;
 syscall_nr_kldstat                     = 308;
 syscall_nr_kldfirstmod                 = 309;
 syscall_nr_kldsym                      = 337;
 syscall_nr_sendfile 			= 393;
 syscall_nr_kse_exit                    = 379;
 syscall_nr_kse_wakeup                  = 380;
 syscall_nr_kse_create                  = 381;
 syscall_nr_kse_thr_interrupt           = 382;
 syscall_nr_kse_release                 = 383;
 syscall_nr_kse_switchin                = 440;
 syscall_nr_getrlimit                   = 194;
 syscall_nr_setrlimit                   = 195;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 syscall_nr_getsid			= 310;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


