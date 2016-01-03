{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2010 Jasper Lievisse Adriaanse
    Copyright (c) 2004 Marco van de Voort
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
 * System call numbers.
 *
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * created from; OpenBSD: syscalls.master,v 1.104 2010/10/28 15:02:41 millert Exp
 *  (OpenBSD/i386 4.9 release)

  Checked with syscall.h release tags from 2.0 to 5.6 2014/10/14

=======
 * created from; OpenBSD: syscalls.master,v 1.104 2010/10/28 15:02:41 millert Exp 
 *  (OpenBSD/i386 4.9 release)
>>>>>>> graemeg/cpstrnew
=======
 * created from; OpenBSD: syscalls.master,v 1.104 2010/10/28 15:02:41 millert Exp 
 *  (OpenBSD/i386 4.9 release)
>>>>>>> graemeg/cpstrnew
=======
 * created from; OpenBSD: syscalls.master,v 1.104 2010/10/28 15:02:41 millert Exp 
 *  (OpenBSD/i386 4.9 release)
>>>>>>> graemeg/cpstrnew
=======
 * created from; OpenBSD: syscalls.master,v 1.104 2010/10/28 15:02:41 millert Exp 
 *  (OpenBSD/i386 4.9 release)
>>>>>>> origin/cpstrnew
}

Const
<<<<<<< HEAD
(* 0 *)    syscall_nr_syscall = 0;
(* 1 *)    syscall_nr_exit = 1;
(* 2 *)    syscall_nr_fork = 2;
(* 3 *)    syscall_nr_read = 3;
(* 4 *)    syscall_nr_write = 4;
(* 5 *)    syscall_nr_open = 5;
(* 6 *)    syscall_nr_close = 6;
(* 7 *)    syscall_nr_wait4 = 7; (* up to 5.4 release *)
(* 7 *)    syscall_nr_getentropy = 7; (* from 5.6 release *)
(* 8 *)    syscall_nr___tfork_52 = 8; (* from 5.2 release *)
(* 9 *)    syscall_nr_link = 9;
(* 10 *)    syscall_nr_unlink = 10;
(* 11 *)    syscall_nr_wait4_55 = 11; (* from 5.5 release *)
(* 12 *)    syscall_nr_chdir = 12;
(* 13 *)    syscall_nr_fchdir = 13;
(* 14 *)    syscall_nr_mknod = 14;
(* 15 *)    syscall_nr_chmod = 15;
(* 16 *)    syscall_nr_chown = 16;
(* 17 *)    syscall_nr_break = 17;
(* 18 *)    syscall_nr_getfsstat_20 = 18; (* up to 2.5 release *)
(* 18 *)    syscall_nr_ogetfsstat = 18; (* from 2.6 up to 2.9 release *)
(* 18 *)    syscall_nr_getdtablecount = 18; (* from 5.2 release *)
(* 19 *)    syscall_nr_getrusage_55 = 19; (* from 5.5 release *)
(* 20 *)    syscall_nr_getpid = 20;
(* 21 *)    syscall_nr_mount = 21;
(* 22 *)    syscall_nr_unmount = 22;
(* 23 *)    syscall_nr_setuid = 23;
(* 24 *)    syscall_nr_getuid = 24;
(* 25 *)    syscall_nr_geteuid = 25;
(* 26 *)    syscall_nr_ptrace = 26;
(* 27 *)    syscall_nr_recvmsg = 27;
(* 28 *)    syscall_nr_sendmsg = 28;
(* 29 *)    syscall_nr_recvfrom = 29;
(* 30 *)    syscall_nr_accept = 30;
(* 31 *)    syscall_nr_getpeername = 31;
(* 32 *)    syscall_nr_getsockname = 32;
(* 33 *)    syscall_nr_access = 33;
(* 34 *)    syscall_nr_chflags = 34;
(* 35 *)    syscall_nr_fchflags = 35;
(* 36 *)    syscall_nr_sync = 36;
(* 37 *)    syscall_nr_kill = 37;
(* 38 *)    syscall_nr_stat_55 = 38; (* from 5.5 release *)
(* 39 *)    syscall_nr_getppid = 39;
(* 40 *)    syscall_nr_lstat_55 = 40; (* from 5.5 release *)
(* 41 *)    syscall_nr_dup = 41;
(* 42 *)    syscall_nr_pipe_20 = 42; (* up to 2.5 release *)
(* 42 *)    syscall_nr_opipe = 42; (* from 2.6 up to 4.9 release *)
(* 42 *)    syscall_nr_fstatat_55 = 42; (* from 5.5 release *)
(* 43 *)    syscall_nr_getegid = 43;
(* 44 *)    syscall_nr_profil = 44;
(* 45 *)    syscall_nr_ktrace = 45;
(* 46 *)    syscall_nr_sigaction = 46;
(* 47 *)    syscall_nr_getgid = 47;
(* 48 *)    syscall_nr_sigprocmask = 48;
(* 49 *)    syscall_nr_getlogin = 49;
(* 50 *)    syscall_nr_setlogin = 50;
(* 51 *)    syscall_nr_acct = 51;
(* 52 *)    syscall_nr_sigpending = 52;
(* 53 *)    syscall_nr_sigaltstack_20 = 53; (* up to 3.4 release *)
(* 53 *)    syscall_nr_osigaltstack = 53; (* from 3.5 up to 4.3 release *)
(* 53 *)    syscall_nr_fstat_55 = 53; (* from 5.5 release *)
(* 54 *)    syscall_nr_ioctl = 54;
(* 55 *)    syscall_nr_reboot = 55;
(* 56 *)    syscall_nr_revoke = 56;
(* 57 *)    syscall_nr_symlink = 57;
(* 58 *)    syscall_nr_readlink = 58;
(* 59 *)    syscall_nr_execve = 59;
(* 60 *)    syscall_nr_umask = 60;
(* 61 *)    syscall_nr_chroot = 61;
(* 62 *)    syscall_nr_getfsstat_54 = 62; (* from 5.4 release *)
(* 63 *)    syscall_nr_statfs_54 = 63; (* from 5.4 release *)
(* 64 *)    syscall_nr_fstatfs_54 = 64; (* from 5.4 release *)
(* 65 *)    syscall_nr_msync_20 = 65; (* up to 2.2 release *)
(* 65 *)    syscall_nr_omsync = 65; (* from 2.3 up to 3.3 release *)
(* 65 *)    syscall_nr_fhstatfs_54 = 65; (* from 5.4 release *)
(* 66 *)    syscall_nr_vfork = 66;
(* 67 *)    syscall_nr_gettimeofday_55 = 67; (* from 5.5 release *)
(* 68 *)    syscall_nr_settimeofday_55 = 68; (* from 5.5 release *)
(* 69 *)    syscall_nr_sbrk = 69; (* up to 4.9 release *)
(* 69 *)    syscall_nr_setitimer_55 = 69; (* from 5.5 release *)
(* 70 *)    syscall_nr_sstk = 70; (* up to 4.9 release *)
(* 70 *)    syscall_nr_getitimer_55 = 70; (* from 5.5 release *)
(* 71 *)    syscall_nr_select_55 = 71; (* from 5.5 release *)
(* 72 *)    syscall_nr_vadvise = 72; (* up to 4.2 release *)
(* 72 *)    syscall_nr_kevent_55 = 72; (* from 5.5 release *)
(* 73 *)    syscall_nr_munmap = 73;
(* 74 *)    syscall_nr_mprotect = 74;
(* 75 *)    syscall_nr_madvise = 75;
(* 76 *)    syscall_nr_utimes_55 = 76; (* from 5.5 release *)
(* 77 *)    syscall_nr_futimes_55 = 77; (* from 5.5 release *)
(* 78 *)    syscall_nr_mincore = 78;
(* 79 *)    syscall_nr_getgroups = 79;
(* 80 *)    syscall_nr_setgroups = 80;
(* 81 *)    syscall_nr_getpgrp = 81;
(* 82 *)    syscall_nr_setpgid = 82;
(* 83 *)    syscall_nr_setitimer_20 = 83; (* up to 5.4 release *)
(* 83 *)    syscall_nr_sendsyslog = 83; (* from 5.6 release *)
(* 84 *)    syscall_nr_utimensat_55 = 84; (* from 5.5 release *)
(* 85 *)    syscall_nr_swapon = 85; (* up to 3.3 release *)
(* 85 *)    syscall_nr_futimens_55 = 85; (* from  5.5 release *)
(* 86 *)    syscall_nr_getitimer_20 = 86;
(* 87 *)    syscall_nr_clock_gettime_55 = 87; (* from  5.5 release *)
(* 88 *)    syscall_nr_clock_settime_55 = 88; (* from  5.5 release *)
(* 89 *)    syscall_nr_clock_getres_55 = 89; (* from  5.5 release *)
(* 90 *)    syscall_nr_dup2 = 90;
(* 91 *)    syscall_nr_nanosleep_55 = 91; (* from  5.5 release *)
(* 92 *)    syscall_nr_fcntl = 92;
(* 93 *)    syscall_nr_select_20 = 93; (* up to 5.4 release *)
(* 94 *)    syscall_nr___thrsleep_55 = 94; (* from 5.5 release *)
(* 95 *)    syscall_nr_fsync = 95;
(* 96 *)    syscall_nr_setpriority = 96;
(* 97 *)    syscall_nr_socket = 97;
(* 98 *)    syscall_nr_connect = 98;
(* 99 *)    syscall_nr_getdents = 99;
(* 100 *)    syscall_nr_getpriority = 100;
(* 103 *)    syscall_nr_sigreturn = 103;
(* 104 *)    syscall_nr_bind = 104;
(* 105 *)    syscall_nr_setsockopt = 105;
(* 106 *)    syscall_nr_listen = 106;
(* 109 *)    syscall_nr_ppoll = 109;
(* 110 *)    syscall_nr_pselect = 110;
(* 111 *)    syscall_nr_sigsuspend = 111;
(* 115 *)    syscall_nr_vtrace = 115;
(* 116 *)    syscall_nr_gettimeofday_20 = 116; (* up to 5.4 release *)
(* 117 *)    syscall_nr_getrusage_20 = 117; (* up to 5.4 release *)
(* 118 *)    syscall_nr_getsockopt = 118;
(* 120 *)    syscall_nr_readv = 120;
(* 121 *)    syscall_nr_writev = 121;
(* 122 *)    syscall_nr_settimeofday_20 = 122; (* up to 5.4 release *)
(* 123 *)    syscall_nr_fchown = 123;
(* 124 *)    syscall_nr_fchmod = 124;
(* 126 *)    syscall_nr_setreuid = 126;
(* 127 *)    syscall_nr_setregid = 127;
(* 128 *)    syscall_nr_rename = 128;
(* 131 *)    syscall_nr_flock = 131;
(* 132 *)    syscall_nr_mkfifo = 132;
(* 133 *)    syscall_nr_sendto = 133;
(* 134 *)    syscall_nr_shutdown = 134;
(* 135 *)    syscall_nr_socketpair = 135;
(* 136 *)    syscall_nr_mkdir = 136;
(* 137 *)    syscall_nr_rmdir = 137;
(* 138 *)    syscall_nr_utimes_20 = 138; (* up to 5.4 release *)
(* 140 *)    syscall_nr_adjtime = 140;
(* 147 *)    syscall_nr_setsid = 147;
(* 148 *)    syscall_nr_quotactl = 148;
(* 155 *)    syscall_nr_nfssvc = 155;
(* 157 *)    syscall_nr_statfs_20 = 157; (* up to 2.5 release *)
(* 157 *)    syscall_nr_ostatfs = 157; (* from 2.6 to 2.9 release *)
(* 158 *)    syscall_nr_fstatfs_20 = 158; (* up to 2.5 release *)
(* 158 *)    syscall_nr_ofstatfs = 158; (* from 2.6 to 2.9 release *)
(* 161 *)    syscall_nr_getfh = 161;
(* 165 *)    syscall_nr_sysarch = 165;
(* 169 *)    syscall_nr_semsys = 169; (* old compat_10, but required for ipcbsd.inc rtl-extra compilation *)
(* 170 *)    syscall_nr_msgsys = 170; (* old compat_10, but required for ipcbsd.inc rtl-extra compilation *)
(* 171 *)    syscall_nr_shmsys = 171; (* old compat_10, but required for ipcbsd.inc rtl-extra compilation *)
(* 173 *)    syscall_nr_pread = 173;
(* 174 *)    syscall_nr_pwrite = 174;
(* 175 *)    syscall_nr_ntp_gettime = 175;
(* 176 *)    syscall_nr_ntp_adjtime = 176;
(* 181 *)    syscall_nr_setgid = 181;
(* 182 *)    syscall_nr_setegid = 182;
(* 183 *)    syscall_nr_seteuid = 183;
(* 184 *)    syscall_nr_lfs_bmapv = 184;
(* 185 *)    syscall_nr_lfs_markv = 185;
(* 186 *)    syscall_nr_lfs_segclean = 186;
(* 187 *)    syscall_nr_lfs_segwait = 187;
(* 188 *)    syscall_nr_stat_20 = 188; (* up to 3.5 release *)
(* 189 *)    syscall_nr_fstat_20 = 189; (* up to 3.5 release *)
(* 190 *)    syscall_nr_lstat_20 = 190; (* up to 3.5 release *)
(* 191 *)    syscall_nr_pathconf = 191;
(* 192 *)    syscall_nr_fpathconf = 192;
(* 193 *)    syscall_nr_swapctl = 193;
(* 194 *)    syscall_nr_getrlimit = 194;
(* 195 *)    syscall_nr_setrlimit = 195;
(* 196 *)    syscall_nr_getdirentries_20 = 196; (* up to 4.8 release *)
(* 196 *)    syscall_nr_ogetdirentries = 196; (* 4.9 release only *)
(* 197 *)    syscall_nr_mmap = 197;
(* 198 *)    syscall_nr___syscall = 198;
(* 199 *)    syscall_nr_lseek = 199;
(* 200 *)    syscall_nr_truncate = 200;
(* 201 *)    syscall_nr_ftruncate = 201;
(* 202 *)    syscall_nr___sysctl = 202;
(* 203 *)    syscall_nr_mlock = 203;
(* 204 *)    syscall_nr_munlock = 204;
(* 205 *)    syscall_nr_undelete = 205;
(* 206 *)    syscall_nr_futimes_20 = 206; (* up to 5.4 release *)
(* 207 *)    syscall_nr_getpgid = 207;
(* 208 *)    syscall_nr_xfspioctl = 208; (* from 2.4 up to 4.5 release *)
(* 208 *)    syscall_nr_nnpfspioctl = 208; (* from 4.6 up to 5.2 release *)
(* 209 *)    syscall_nr_utrace = 209;
(* 220 *)    syscall_nr___semctl_20 = 220; (* up to 2.3 release *)
(* 220 *)    syscall_nr___osemctl = 220; (* from 2.4 up to 2.9 release *)
(* 221 *)    syscall_nr_semget = 221;
(* 222 *)    syscall_nr_semop_20 = 222;
(* 223 *)    syscall_nr_semconfig = 223;
(* 224 *)    syscall_nr_msgctl_20 = 224; (* up to 2.3 release *)
(* 224 *)    syscall_nr_omsgctl = 224; (* from 2.4 up to 2.9 release *)
(* 225 *)    syscall_nr_msgget = 225;
(* 226 *)    syscall_nr_msgsnd = 226;
(* 227 *)    syscall_nr_msgrcv = 227;
(* 228 *)    syscall_nr_shmat = 228;
(* 229 *)    syscall_nr_shmctl_20 = 229; (* up to 2.3 release *)
(* 229 *)    syscall_nr_oshmctl = 229; (* from 2.4 up to 2.9 release *)
(* 230 *)    syscall_nr_shmdt = 230;
(* 231 *)    syscall_nr_shmget_20 = 231; (* up to 3.5 release *)
(* 232 *)    syscall_nr_clock_gettime_20 = 232; (* up to 5.4 release *)
(* 233 *)    syscall_nr_clock_settime_20 = 233; (* up to 5.4 release *)
(* 234 *)    syscall_nr_clock_getres_20 = 234; (* up to 5.4 release *)
(* 240 *)    syscall_nr_nanosleep_20 = 240; (* up to 5.4 release *)
(* 250 *)    syscall_nr_minherit = 250;
(* 251 *)    syscall_nr_rfork = 251;
(* 252 *)    syscall_nr_poll = 252;
(* 253 *)    syscall_nr_issetugid = 253;
(* 254 *)    syscall_nr_lchown = 254;
(* 255 *)    syscall_nr_getsid = 255;
(* 256 *)    syscall_nr_msync = 256;
(* 257 *)    syscall_nr___semctl_24 = 257; (* from 2.4 up to 3.5 release *)
(* 258 *)    syscall_nr_shmctl_24 = 258; (* from 2.4 up to 3.5 release *)
(* 259 *)    syscall_nr_msgctl_24 = 259; (* from 2.4 up to 3.5 release *)
(* 260 *)    syscall_nr_getfsstat_26 = 260; (* from 2.6 up to 4.3 release *)
(* 261 *)    syscall_nr_statfs_26 = 261; (* from 2.6 up to 4.3 release *)
(* 262 *)    syscall_nr_fstatfs_26 = 262; (* from 2.6 up to 4.3 release *)
(* 263 *)    syscall_nr_pipe_26 = 263; (* from 2.6 release *)
(* 264 *)    syscall_nr_fhopen = 264;
(* 265 *)    syscall_nr_fhstat_20 = 265; (* up to 3.5 release *)
(* 266 *)    syscall_nr_fhstatfs_20 = 266; (* up to 4.4 release *)
(* 267 *)    syscall_nr_preadv = 267;
(* 268 *)    syscall_nr_pwritev = 268;
(* 269 *)    syscall_nr_kqueue = 269;
(* 270 *)    syscall_nr_kevent_29 = 270; (*from 2.9 up to 5.4 release *)
(* 271 *)    syscall_nr_mlockall = 271;
(* 272 *)    syscall_nr_munlockall = 272;
(* 273 *)    syscall_nr_getpeereid = 273;
(* 274 *)    syscall_nr_extattrctl = 274;
(* 275 *)    syscall_nr_extattr_set_file = 275;
(* 276 *)    syscall_nr_extattr_get_file = 276;
(* 277 *)    syscall_nr_extattr_delete_file = 277;
(* 278 *)    syscall_nr_extattr_set_fd = 278;
(* 279 *)    syscall_nr_extattr_get_fd = 279;
(* 280 *)    syscall_nr_extattr_delete_fd = 280;
(* 281 *)    syscall_nr_getresuid = 281;
(* 282 *)    syscall_nr_setresuid = 282;
(* 283 *)    syscall_nr_getresgid = 283;
(* 284 *)    syscall_nr_setresgid = 284;
(* 286 *)    syscall_nr_mquery = 286;
(* 287 *)    syscall_nr_closefrom = 287;
(* 288 *)    syscall_nr_sigaltstack_35 = 288; (* from 3.5 release *)
(* 289 *)    syscall_nr_shmget_36 = 289; (* from 3.6 release *)
(* 290 *)    syscall_nr_semop_36 = 290; (* from 3.6 release *)
(* 291 *)    syscall_nr_stat_36 = 291; (* from 3.6 up to 5.4 release *)
(* 292 *)    syscall_nr_fstat_36 = 292; (* from 3.6 up to 5.4 release *)
(* 293 *)    syscall_nr_lstat_36 = 293; (* from 3.6 up to 5.4 release *)
(* 294 *)    syscall_nr_fhstat_36 = 294; (* from 3.6 release *)
(* 295 *)    syscall_nr___semctl_36 = 295; (* from 3.6 release *)
(* 296 *)    syscall_nr_shmctl_36 = 296; (* from 3.6 release *)
(* 297 *)    syscall_nr_msgctl_36 = 297; (* from 3.6 release *)
(* 298 *)    syscall_nr_sched_yield = 298;
(* 299 *)    syscall_nr_getthrid = 299;
(* 300 *)    syscall_nr_thrsleep = 300; (* from 3.9 up to 5.0 release *)
(* 300 *)    syscall_nr___thrsleep_51 = 300; (* from 5.1 up to 5.4 release *)
(* 301 *)    syscall_nr_thrwakeup = 301; (* from 3.9 up to 5.0 release *)
(* 301 *)    syscall_nr___thrwakeup = 301; (* from 5.1 release *)
(* 302 *)    syscall_nr_threxit = 302; (* from 3.9 up to 5.0 release *)
(* 302 *)    syscall_nr___threxit = 302; (* from 5.1 release *)
(* 303 *)    syscall_nr_thrsigdivert = 303; (* from 3.9 up to 5.0 release *)
(* 303 *)    syscall_nr___thrsigdivert = 303; (* from 5.1 release *)
(* 304 *)    syscall_nr___getcwd = 304;
(* 305 *)    syscall_nr_adjfreq = 305;
(* 306 *)    syscall_nr_getfsstat_44 = 306; (* from 4.4 up to 5.3 release *)
(* 307 *)    syscall_nr_statfs_44 = 307; (* from 4.4 up to 5.3 release *)
(* 308 *)    syscall_nr_fstatfs_44 = 308; (* from 4.4 up to 5.3 release *)
(* 309 *)    syscall_nr_fhstatfs_44 = 309; (* from 4.4 up to 5.3 release *)
(* 310 *)    syscall_nr_setrdomain = 310; (* 4.7 release only *)
(* 310 *)    syscall_nr_setrtable = 310; (* from 4.8 release *)
(* 311 *)    syscall_nr_getrdomain = 311; (* 4.7 release only *)
(* 311 *)    syscall_nr_getrtable = 311; (* from 4.8 release *)
(* 312 *)    syscall_nr_getdirentries_49 = 312; (* from 4.9 up to 5.4 release *)
(* 313 *)    syscall_nr_faccessat = 313;
(* 314 *)    syscall_nr_fchmodat = 314;
(* 315 *)    syscall_nr_fchownat = 315;
(* 316 *)    syscall_nr_fstatat_50 = 316; (* from 5.0 release *)
(* 317 *)    syscall_nr_linkat = 317;
(* 318 *)    syscall_nr_mkdirat = 318;
(* 319 *)    syscall_nr_mkfifoat = 319;
(* 320 *)    syscall_nr_mknodat = 320;
(* 321 *)    syscall_nr_openat = 321;
(* 322 *)    syscall_nr_readlinkat = 322;
(* 323 *)    syscall_nr_renameat = 323;
(* 324 *)    syscall_nr_symlinkat = 324;
(* 325 *)    syscall_nr_unlinkat = 325;
(* 326 *)    syscall_nr_utimensat_50 = 326;
(* 327 *)    syscall_nr_futimens_50 = 327;
(* 328 *)    syscall_nr___tfork_51 = 328; (* 5.1 release only *)
(* 329 *)    syscall_nr___set_tcb = 329;
(* 330 *)    syscall_nr___get_tcb = 330;


{ Aliases }
var
  syscall_nr_waitpid : longint = syscall_nr_wait4;  // 7, added: ease of notation purposes
  
  syscall_nr_setitimer : longint = syscall_nr_setitimer_20;
  syscall_nr_getitimer : longint = syscall_nr_getitimer_20;
  syscall_nr_select : longint = syscall_nr_select_20;
  syscall_nr_gettimeofday : longint = syscall_nr_gettimeofday_20;
  syscall_nr_settimeofday : longint = syscall_nr_settimeofday_20;
  syscall_nr_clock_gettime : longint = syscall_nr_clock_gettime_20;
  syscall_nr_clock_settime : longint = syscall_nr_clock_settime_20;
  syscall_nr_clock_getres : longint = syscall_nr_clock_getres_20;
  syscall_nr_getrusage : longint = syscall_nr_getrusage_20;
  syscall_nr_utimes : longint = syscall_nr_utimes_20;
  syscall_nr_futimes : longint = syscall_nr_futimes_20;
  syscall_nr_statfs : longint = syscall_nr_statfs_26;
  syscall_nr_fstatfs : longint = syscall_nr_fstatfs_26;
  syscall_nr_stat : longint = syscall_nr_stat_36;
  syscall_nr_fstat : longint = syscall_nr_fstat_36;
  syscall_nr_lstat : longint = syscall_nr_lstat_36;
  syscall_nr_fhstat : longint = syscall_nr_fhstat_36;
  syscall_nr_fstatat : longint = syscall_nr_fstatat_50;
  syscall_nr_fhstatfs : longint = syscall_nr_fhstatfs_20;
  syscall_nr_nanosleep : longint = syscall_nr_nanosleep_20;
  syscall_nr_kevent : longint = syscall_nr_kevent_29;
  syscall_nr_pipe : longint = syscall_nr_pipe_26;
  syscall_nr_sigaltstack : longint = syscall_nr_sigaltstack_35;
  syscall_nr___tfork : longint = -1;
  syscall_nr_msgctl : longint = syscall_nr_msgctl_36;
  syscall_nr_semop : longint = syscall_nr_semop_36;
  syscall_nr___semctl : longint = syscall_nr___semctl_36;
  syscall_nr_shmctl : longint = syscall_nr_shmctl_36;
  syscall_nr_shmget : longint = syscall_nr_shmget_36;
  syscall_nr_getdirentries : longint = syscall_nr_getdirentries_20;
{ The variables above need to be changed at startup }

{$define FPC_HAS_SETSYSNR_INC}
{$define FPC_USE_GETDIRENTRIES_SYSCALL}
{$define FPC_USE_GETDIRENTRIES_I49_SYSCALL}

=======
        syscall_nr_syscall      = 0  ;
        syscall_nr_exit         = 1  ;
        syscall_nr_fork         = 2  ;
        syscall_nr_read         = 3  ;
        syscall_nr_write        = 4  ;
        syscall_nr_open         = 5  ;
        syscall_nr_close        = 6  ;
        syscall_nr_wait4        = 7  ;
        syscall_nr_waitpid      = 8  ;  // added: ease of notation purposes
        syscall_nr_link         = 9  ;
        syscall_nr_unlink       = 10 ;
        syscall_nr_chdir        = 12 ;
        syscall_nr_fchdir       = 13 ;
        syscall_nr_mknod        = 14 ;
        syscall_nr_chmod        = 15 ;
        syscall_nr_chown        = 16 ;
        syscall_nr_break        = 17 ;
        syscall_nr_getpid       = 20 ;
        syscall_nr_mount        = 21 ;
        syscall_nr_unmount      = 22 ;
        syscall_nr_setuid       = 23 ;
        syscall_nr_getuid       = 24 ;
        syscall_nr_geteuid      = 25 ;
        syscall_nr_ptrace       = 26 ;
        syscall_nr_recvmsg      = 27 ;
        syscall_nr_sendmsg      = 28 ;
        syscall_nr_recvfrom     = 29 ;
        syscall_nr_accept       = 30 ;
        syscall_nr_getpeername  = 31 ;
        syscall_nr_getsockname  = 32 ;
        syscall_nr_access       = 33 ;
        syscall_nr_chflags      = 34 ;
        syscall_nr_fchflags     = 35 ;
        syscall_nr_sync         = 36 ;
        syscall_nr_kill         = 37 ;
        syscall_nr_getppid      = 39 ;
        syscall_nr_dup          = 41 ;
        syscall_nr_opipe        = 42 ;
        syscall_nr_getegid      = 43 ;
        syscall_nr_profil       = 44 ;
        syscall_nr_ktrace       = 45 ;
        syscall_nr_sigaction    = 46 ;
        syscall_nr_getgid       = 47 ;
        syscall_nr_sigprocmask  = 48 ;
        syscall_nr_getlogin     = 49 ;
        syscall_nr_setlogin     = 50 ;
        syscall_nr_acct         = 51 ;
        syscall_nr_sigpending   = 52 ;
        syscall_nr_sigaltstack  = 53 ;
        syscall_nr_ioctl        = 54 ;
        syscall_nr_reboot       = 55 ;
        syscall_nr_revoke       = 56 ;
        syscall_nr_symlink      = 57 ;
        syscall_nr_readlink     = 58 ;
        syscall_nr_execve       = 59 ;
        syscall_nr_umask        = 60 ;
        syscall_nr_chroot       = 61 ;
        syscall_nr_vfork        = 66 ;
        syscall_nr_sbrk         = 69 ;
        syscall_nr_sstk         = 70 ;
        syscall_nr_vadvise      = 72 ;
        syscall_nr_munmap       = 73 ;
        syscall_nr_mprotect     = 74 ;
        syscall_nr_madvise      = 75 ;
        syscall_nr_mincore      = 78 ;
        syscall_nr_getgroups    = 79 ;
        syscall_nr_setgroups    = 80 ;
        syscall_nr_getpgrp      = 81 ;
        syscall_nr_setpgid      = 82 ;
        syscall_nr_setitimer    = 83 ;
        syscall_nr_getitimer    = 86 ;
        syscall_nr_dup2         = 90 ;
        syscall_nr_fcntl        = 92 ;
        syscall_nr_select       = 93 ;
        syscall_nr_fsync        = 95 ;
        syscall_nr_setpriority  = 96 ;
        syscall_nr_socket       = 97 ;
        syscall_nr_connect      = 98 ;
        syscall_nr_getpriority  = 100;
        syscall_nr_sigreturn    = 103;
        syscall_nr_bind         = 104;
        syscall_nr_setsockopt   = 105;
        syscall_nr_listen       = 106;
        syscall_nr_sigsuspend   = 111;
        syscall_nr_gettimeofday = 116;
        syscall_nr_getrusage    = 117;
        syscall_nr_getsockopt   = 118;
        syscall_nr_readv        = 120;
        syscall_nr_writev       = 121;
        syscall_nr_settimeofday = 122;
        syscall_nr_fchown       = 123;
        syscall_nr_fchmod       = 124;
        syscall_nr_setreuid     = 126;
        syscall_nr_setregid     = 127;
        syscall_nr_rename       = 128;
        syscall_nr_flock        = 131;
        syscall_nr_mkfifo       = 132;
        syscall_nr_sendto       = 133;
        syscall_nr_shutdown     = 134;
        syscall_nr_socketpair   = 135;
        syscall_nr_mkdir        = 136;
        syscall_nr_rmdir        = 137;
        syscall_nr_utimes       = 138;
        syscall_nr_adjtime      = 140;
        syscall_nr_setsid       = 147;
        syscall_nr_quotactl     = 148;
        syscall_nr_nfssvc       = 155;
        syscall_nr_getfh        = 161;
        syscall_nr_sysarch      = 165;
        syscall_nr_pread        = 173;
        syscall_nr_pwrite       = 174;
        syscall_nr_setgid       = 181;
        syscall_nr_setegid      = 182;
        syscall_nr_seteuid      = 183;
        syscall_nr_lfs_bmapv    = 184;
        syscall_nr_lfs_markv    = 185;
        syscall_nr_lfs_segclean = 186;
        syscall_nr_lfs_segwait  = 187;
        syscall_nr_stat         = 188;
        syscall_nr_fstat        = 189;
        syscall_nr_lstat        = 190;
        syscall_nr_pathconf     = 191;
        syscall_nr_fpathconf    = 192;
        syscall_nr_swapctl      = 193;
        syscall_nr_getrlimit    = 194;
        syscall_nr_setrlimit    = 195;
        syscall_nr_getdirentries= 196;
        syscall_nr_mmap         = 197;
        syscall_nr___syscall    = 198;
        syscall_nr_lseek        = 199;
        syscall_nr_truncate     = 200;
        syscall_nr_ftruncate    = 201;
        syscall_nr___sysctl     = 202;
        syscall_nr_mlock        = 203;
        syscall_nr_munlock      = 204;
        syscall_nr_undelete     = 205;
        syscall_nr_futimes      = 206;
        syscall_nr_getpgid      = 207;
        syscall_nr_xfspioctl    = 208;
        syscall_nr_semget       = 221;
        syscall_nr_semop        = 222;
        syscall_nr_msgget       = 225;
        syscall_nr_msgsnd       = 226;
        syscall_nr_msgrcv       = 227;
        syscall_nr_shmat        = 228;
        syscall_nr_shmdt        = 230;
        syscall_nr_shmget       = 231;
        syscall_nr_clock_gettime= 232;
        syscall_nr_clock_settime= 233;
        syscall_nr_clock_getres = 234;
        syscall_nr_nanosleep    = 240;
        syscall_nr_minherit     = 250;
        syscall_nr_rfork        = 251;
        syscall_nr_poll         = 252;
        syscall_nr_issetugid    = 253;
        syscall_nr_lchown       = 254;
        syscall_nr_getsid       = 255;
        syscall_nr_msync        = 256;
        syscall_nr___semctl     = 257;
        syscall_nr_shmctl       = 258;
        syscall_nr_msgctl       = 259;
        syscall_nr_getfsstat    = 260;
        syscall_nr_statfs       = 261;
        syscall_nr_fstatfs      = 262;
        syscall_nr_pipe         = 263;
        syscall_nr_fhopen       = 264;
        syscall_nr_fhstat       = 265;
        syscall_nr_fhstatfs     = 266;
        syscall_nr_preadv       = 267;
        syscall_nr_pwritev      = 268;
        syscall_nr_kqueue       = 269;
        syscall_nr_kevent       = 270;
        syscall_nr_mlockall     = 271;
        syscall_nr_munlockall   = 272;
        syscall_nr_getpeereid   = 273;
        syscall_nr_extattrctl   = 274;
        syscall_nr_extattr_set_file     = 275;
        syscall_nr_extattr_get_file     = 276;
        syscall_nr_extattr_delete_file  = 277;
        syscall_nr_extattr_set_fd       = 278;
        syscall_nr_extattr_get_fd       = 279;
        syscall_nr_extattr_delete_fd    = 280;
        syscall_nr_getresuid    = 281;
        syscall_nr_setresuid    = 282;
        syscall_nr_getresgid    = 283;
        syscall_nr_setresgid    = 284;
        syscall_nr_mquery       = 286;
        syscall_nr_closefrom    = 287;
        syscall_nr_signalstack  = 288;
        syscall_nr_shmget       = 289;
        syscall_nr_semop        = 290;
        syscall_nr_stat         = 291;
        syscall_nr_fstat        = 292;
        syscall_nr_lstat        = 293;
        syscall_nr_fhstat       = 294;
        syscall_nr___semctl     = 294;
        syscall_nr_shmctl       = 295;
        syscall_nr_msgctl       = 296;
        syscall_nr_sched_yield  = 298;
        syscall_nr_getthrid     = 299;
        syscall_nr_thrsleep     = 300;
        syscall_nr_thrwakeup    = 301;
        syscall_nr_exit         = 302;
        syscall_nr_thrsigdivert = 303;
        syscall_nr___getcwd     = 304;
        syscall_nr_adjfreq      = 305;
        syscall_nr_getfsstat    = 306;
        syscall_nr_statfs       = 307;
        syscall_nr_fstatfs      = 308;
        syscall_nr_fhstatfs     = 309;
        syscall_nr_setrtable    = 310;
        syscall_nr_getrtable    = 311;
        syscall_nr_getdirentries        = 312;
        syscall_nr_MAXSYSCALL   = 313;
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
