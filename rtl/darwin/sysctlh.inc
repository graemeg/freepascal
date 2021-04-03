{
}

{$PACKRECORDS C}

  {
     Copyright (c) 2000-2003 Apple Computer, Inc. All rights reserved.

     @APPLE_LICENSE_HEADER_START@

     The contents of this file constitute Original Code as defined in and
     are subject to the Apple Public Source License Version 1.1 (the
     "License").  You may not use this file except in compliance with the
     License.  Please obtain a copy of the License at
     http://www.apple.com/publicsource and read it before using this file.

     This Original Code and all software distributed under the License are
     distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
     EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
     INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
     FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
     License for the specific language governing rights and limitations
     under the License.

     @APPLE_LICENSE_HEADER_END@
    }
  { Copyright (c) 1995 NeXT Computer, Inc. All Rights Reserved  }
  {
     Copyright (c) 1989, 1993
        The Regents of the University of California.  All rights reserved.

     This code is derived from software contributed to Berkeley by
     Mike Karels at Berkeley Software Design, Inc.

     Redistribution and use in source and binary forms, with or without
     modification, are permitted provided that the following conditions
     are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. All advertising materials mentioning features or use of this software
        must display the following acknowledgement:
        This product includes software developed by the University of
        California, Berkeley and its contributors.
     4. Neither the name of the University nor the names of its contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.

     THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
     FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
     OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
     OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
     SUCH DAMAGE.

        @(#)sysctl.h    8.1 (Berkeley) 6/2/93
    }
  {
     These are for the eproc structure defined below.
    }
  {
     Definitions for sysctl call.  The sysctl call uses a hierarchical name
     for objects that can be examined or modified.  The name is expressed as
     a sequence of integers.  Like a file path name, the meaning of each
     component depends on its place in the hierarchy.  The top-level and kern
     identifiers are defined here, and other identifiers are defined in the
     respective subsystem header files.
    }
  { largest number of components supported  }

  const
     CTL_MAXNAME = 12;
  {
     Each subsystem defined by sysctl defines a list of variables
     for that subsystem. Each name is either a node with further
     levels defined below it, or it is a leaf of some particular
     type given below. Each sysctl level defines a set of name/type
     pairs to be used by sysctl(1) in manipulating the subsystem.
    }
  { subsystem name  }
  { type of name  }

  type
     ctlname = record
          ctl_name : ^char;
          ctl_type : longint;
       end;

  { Mask for the type  }

  const
     CTLTYPE = $f;
  { name is a node  }
     CTLTYPE_NODE = 1;
  { name describes an integer  }
     CTLTYPE_INT = 2;
  { name describes a string  }
     CTLTYPE_STRING = 3;
  { name describes a 64-bit number  }
     CTLTYPE_QUAD = 4;
  { name describes a structure  }
     CTLTYPE_OPAQUE = 5;
  { name describes a structure  }
     CTLTYPE_STRUCT = CTLTYPE_OPAQUE;
  { Allow reads of variable  }
     CTLFLAG_RD = $80000000;
  { Allow writes to the variable  }
     CTLFLAG_WR = $40000000;
     CTLFLAG_RW = CTLFLAG_RD or CTLFLAG_WR;
  { XXX Don't Lock  }
     CTLFLAG_NOLOCK = $20000000;
  { All users can set this var  }
     CTLFLAG_ANYBODY = $10000000;
  { Permit set only if securelevel<=0  }
     CTLFLAG_SECURE = $08000000;
  { deprecated variable, do not display  }
     CTLFLAG_MASKED = $04000000;
  { do not auto-register  }
     CTLFLAG_NOAUTO = $02000000;
  { valid inside the kernel  }
     CTLFLAG_KERN = $01000000;
  {
     USE THIS instead of a hardwired number from the categories below
     to get dynamically assigned sysctl entries using the linker-set
     technology. This is the way nearly all new sysctl variables should
     be implemented.
     e.g. SYSCTL_INT(_parent, OID_AUTO, name, CTLFLAG_RW, &variable, 0, "");
    }
     OID_AUTO = -(1);
  {
     Top-level identifiers
    }
  { unused  }
     CTL_UNSPEC = 0;
  { "high kernel": proc, limits  }
     CTL_KERN = 1;
  { virtual memory  }
     CTL_VM = 2;
  { file system, mount type is next  }
     CTL_VFS = 3;
  { network, see socket.h  }
     CTL_NET = 4;
  { debugging parameters  }
     CTL_DEBUG = 5;
  { generic cpu/io  }
     CTL_HW = 6;
  { machine dependent  }
     CTL_MACHDEP = 7;
  { user-level  }
     CTL_USER = 8;
  { number of valid top-level ids  }
     CTL_MAXID = 9;
    {
       CTL_KERN identifiers
      }
    { string: system version  }
       KERN_OSTYPE = 1;
    { string: system release  }
       KERN_OSRELEASE = 2;
    { int: system revision  }
       KERN_OSREV = 3;
    { string: compile time info  }
       KERN_VERSION = 4;
    { int: max vnodes  }
       KERN_MAXVNODES = 5;
    { int: max processes  }
       KERN_MAXPROC = 6;
    { int: max open files  }
       KERN_MAXFILES = 7;
    { int: max arguments to exec  }
       KERN_ARGMAX = 8;
    { int: system security level  }
       KERN_SECURELVL = 9;
    { string: hostname  }
       KERN_HOSTNAME = 10;
    { int: host identifier  }
       KERN_HOSTID = 11;
    { struct: struct clockrate  }
       KERN_CLOCKRATE = 12;
    { struct: vnode structures  }
       KERN_VNODE = 13;
    { struct: process entries  }
       KERN_PROC = 14;
    { struct: file entries  }
       KERN_FILE = 15;
    { node: kernel profiling info  }
       KERN_PROF = 16;
    { int: POSIX.1 version  }
       KERN_POSIX1 = 17;
    { int: # of supplemental group ids  }
       KERN_NGROUPS = 18;
    { int: is job control available  }
       KERN_JOB_CONTROL = 19;
    { int: saved set-user/group-ID  }
       KERN_SAVED_IDS = 20;
    { struct: time kernel was booted  }
       KERN_BOOTTIME = 21;
    { string: YP domain name  }
       KERN_NISDOMAINNAME = 22;
       KERN_DOMAINNAME = KERN_NISDOMAINNAME;
    { int: number of partitions/disk  }
       KERN_MAXPARTITIONS = 23;
    { int: kernel trace points  }
       KERN_KDEBUG = 24;
    { int: update process sleep time  }
       KERN_UPDATEINTERVAL = 25;
    { int: OS release date  }
       KERN_OSRELDATE = 26;
    { node: NTP PLL control  }
       KERN_NTP_PLL = 27;
    { string: name of booted kernel  }
       KERN_BOOTFILE = 28;
    { int: max open files per proc  }
       KERN_MAXFILESPERPROC = 29;
    { int: max processes per uid  }
       KERN_MAXPROCPERUID = 30;
    { dev_t: device to dump on  }
       KERN_DUMPDEV = 31;
    { node: anything related to IPC  }
       KERN_IPC = 32;
    { unused  }
       KERN_DUMMY = 33;
    { int: address of PS_STRINGS  }
       KERN_PS_STRINGS = 34;
    { int: address of USRSTACK  }
       KERN_USRSTACK = 35;
    { int: do we log sigexit procs?  }
       KERN_LOGSIGEXIT = 36;
    { string: kernel symbol filename  }
       KERN_SYMFILE = 37;
       KERN_PROCARGS = 38;
    { node: pc sampling  }
       KERN_PCSAMPLES = 39;
    { int: are we netbooted? 1=yes,0=no  }
       KERN_NETBOOT = 40;
    { node: panic UI information  }
       KERN_PANICINFO = 41;
    { node: panic UI information  }
       KERN_SYSV = 42;
    { xxx  }
       KERN_AFFINITY = 43;
    { xxx  }
       KERN_CLASSIC = 44;
    { xxx  }
       KERN_CLASSICHANDLER = 45;
    { int: max aio requests  }
       KERN_AIOMAX = 46;
    { int: max aio requests per process  }
       KERN_AIOPROCMAX = 47;
    { int: max aio worker threads  }
       KERN_AIOTHREADS = 48;
    { __APPLE_API_UNSTABLE  }
    { number of valid kern ids  }

    const
       KERN_MAXID = 50;
    { KERN_KDEBUG types  }
       KERN_KDEFLAGS = 1;
       KERN_KDDFLAGS = 2;
       KERN_KDENABLE = 3;
       KERN_KDSETBUF = 4;
       KERN_KDGETBUF = 5;
       KERN_KDSETUP = 6;
       KERN_KDREMOVE = 7;
       KERN_KDSETREG = 8;
       KERN_KDGETREG = 9;
       KERN_KDREADTR = 10;
       KERN_KDPIDTR = 11;
       KERN_KDTHRMAP = 12;
    { Don't use 13 as it is overloaded with KERN_VNODE  }
       KERN_KDPIDEX = 14;
       KERN_KDSETRTCDEC = 15;
       KERN_KDGETENTROPY = 16;
    { KERN_PCSAMPLES types  }
       KERN_PCDISABLE = 1;
       KERN_PCSETBUF = 2;
       KERN_PCGETBUF = 3;
       KERN_PCSETUP = 4;
       KERN_PCREMOVE = 5;
       KERN_PCREADBUF = 6;
       KERN_PCSETREG = 7;
       KERN_PCCOMM = 8;
    { KERN_PANICINFO types  }
    { quad: panic UI image size limit  }
       KERN_PANICINFO_MAXSIZE = 1;
    { string: path to the panic UI (16 bit)  }
       KERN_PANICINFO_IMAGE16 = 2;
    { string: path to the panic UI (32 bit)  }
       KERN_PANICINFO_IMAGE32 = 3;
    {
       KERN_SYSV identifiers
      }
    { int: max shared memory segment size (bytes)  }
       KSYSV_SHMMAX = 1;
    { int: min shared memory segment size (bytes)  }
       KSYSV_SHMMIN = 2;
    { int: max number of shared memory identifiers  }
       KSYSV_SHMMNI = 3;
    { int: max shared memory segments per process  }
       KSYSV_SHMSEG = 4;
    { int: max amount of shared memory (pages)  }
       KSYSV_SHMALL = 5;
    { int: max num of semaphore identifiers   }
       KSYSV_SEMMNI = 6;
    { int: max num of semaphores in system  }
       KSYSV_SEMMNS = 7;
    { int: max num of undo structures in system   }
       KSYSV_SEMMNU = 8;
    { int: max num of semaphores per id   }
       KSYSV_SEMMSL = 9;
    { int: max num of undo entries per process  }
       KSYSV_SEMUNE = 10;
       error1 = 1;
    {
       KERN_PROC subtypes
      }
    { everything  }
       KERN_PROC_ALL = 0;
    { by process id  }
       KERN_PROC_PID = 1;
    { by process group id  }
       KERN_PROC_PGRP = 2;
    { by session of pid  }
       KERN_PROC_SESSION = 3;
    { by controlling tty  }
       KERN_PROC_TTY = 4;
    { by effective uid  }
       KERN_PROC_UID = 5;
    { by real uid  }
       KERN_PROC_RUID = 6;
    {
       KERN_PROC subtype ops return arrays of augmented proc structures:
      }
    {
       KERN_IPC identifiers
      }
    { int: max size of a socket buffer  }

    const
       KIPC_MAXSOCKBUF = 1;
    { int: wastage factor in sockbuf  }
       KIPC_SOCKBUF_WASTE = 2;
    { int: max length of connection q  }
       KIPC_SOMAXCONN = 3;
    { int: max length of link header  }
       KIPC_MAX_LINKHDR = 4;
    { int: max length of network header  }
       KIPC_MAX_PROTOHDR = 5;
    { int: max total length of headers  }
       KIPC_MAX_HDR = 6;
    { int: max length of data?  }
       KIPC_MAX_DATALEN = 7;
    { struct: mbuf usage statistics  }
       KIPC_MBSTAT = 8;
    { int: maximum mbuf clusters  }
       KIPC_NMBCLUSTERS = 9;
    {
       CTL_VM identifiers
      }
    { struct vmmeter  }
       VM_METER = 1;
    { struct loadavg  }
       VM_LOADAVG = 2;
    {  Note: "3" was skipped sometime ago and should probably remain unused 
             to avoid any new entry from being accepted by older kernels...}
    { struct loadavg with mach factor }
       VM_MACHFACTOR = 4;
    { total swap usage }
       VM_SWAPUSAGE = 5;
    { number of valid vm ids  }
       VM_MAXID = 6;
    {
       CTL_HW identifiers
      }
    { string: machine class  }
       HW_MACHINE = 1;
    { string: specific machine model  }
       HW_MODEL = 2;
    { int: number of cpus  }
       HW_NCPU = 3;
    { int: machine byte order  }
       HW_BYTEORDER = 4;
    { int: total memory  }
       HW_PHYSMEM = 5;
    { int: non-kernel memory  }
       HW_USERMEM = 6;
    { int: software page size  }
       HW_PAGESIZE = 7;
    { strings: disk drive names  }
       HW_DISKNAMES = 8;
    { struct: diskstats[]  }
       HW_DISKSTATS = 9;
    { int: 0 for Legacy, else NewWorld  }
       HW_EPOCH = 10;
    { int: has HW floating point?  }
       HW_FLOATINGPT = 11;
    { string: machine architecture  }
       HW_MACHINE_ARCH = 12;
    { int: has HW vector unit?  }
       HW_VECTORUNIT = 13;
    { int: Bus Frequency  }
       HW_BUS_FREQ = 14;
    { int: CPU Frequency  }
       HW_CPU_FREQ = 15;
    { int: Cache Line Size in Bytes  }
       HW_CACHELINE = 16;
    { int: L1 I Cache Size in Bytes  }
       HW_L1ICACHESIZE = 17;
    { int: L1 D Cache Size in Bytes  }
       HW_L1DCACHESIZE = 18;
    { int: L2 Cache Settings  }
       HW_L2SETTINGS = 19;
    { int: L2 Cache Size in Bytes  }
       HW_L2CACHESIZE = 20;
    { int: L3 Cache Settings  }
       HW_L3SETTINGS = 21;
    { int: L3 Cache Size in Bytes  }
       HW_L3CACHESIZE = 22;
    { int: Bus Frequency  }
       HW_TB_FREQ = 23;
    { uint64_t: physical ram size  }
       HW_MEMSIZE = 24;
    { int: number of available CPUs  }
       HW_AVAILCPU = 25;
    { number of valid hw ids  }
       HW_MAXID = 26;

    {
       These are the support HW selectors for sysctlbyname.  Parameters that are byte count or frequencies are 64 bit numbers.
       All other parameters are 32 bit numbers.

         hw.memsize                - The number of bytes of physical memory in the system.

         hw.ncpu                   - The number maximum number of processor that could be available this boot.
                                     Use this value for sizing of static per processor arrays; i.e. processor load statistics.

         hw.activecpu              - The number of cpus currently available for executing threads.
                                     Use this number to determine the number threads to create in SMP aware applications.
                                     This number can change when power management modes are changed.

         hw.tbfrequency            - This gives the time base frequency used by the OS and is the basis of all timing services.
                                     In general is is better to use mach's or higher level timing services, but this value
                                     is needed to convert the PPC Time Base registers to real time.

         hw.cpufrequency           - These values provide the current, min and max cpu frequency.  The min and max are for
         hw.cpufrequency_max       - all power management modes.  The current frequency is the max frequency in the current mode.
         hw.cpufrequency_min       - All frequencies are in Hz.

         hw.busfrequency           - These values provide the current, min and max bus frequency.  The min and max are for
         hw.busfrequency_max       - all power management modes.  The current frequency is the max frequency in the current mode.
         hw.busfrequency_min       - All frequencies are in Hz.

         hw.cputype                - These values provide the mach-o cpu type and subtype.  A complete list is in <mach/machine.h>
         hw.cpusubtype             - These values should be used to determine what processor family the running cpu is from so that
                                     the best binary can be chosen, or the best dynamic code generated.  They should not be used
                                     to determine if a given processor feature is available.

         hw.byteorder              - Gives the byte order of the processor.  4321 for big endian, 1234 for little.

         hw.pagesize               - Gives the size in bytes of the pages used by the processor and VM system.

         hw.cachelinesize          - Gives the size in bytes of the processor's cache lines.
                                     This value should be use to control the strides of loops that use cache control instructions
                                     like dcbz, dcbt or dcbst.

         hw.l1dcachesize           - These values provide the size in bytes of the L1, L2 and L3 caches.  If a cache is not present
         hw.l1icachesize           - then the selector will return and error.
         hw.l2cachesize            -
         hw.l3cachesize            -


       These are the selectors for optional processor features.  Selectors that return errors are not support on the system.
       Supported features will return 1 if they are recommended or 0 if they are supported but are not expected to help performance.
       Future versions of these selectors may return larger values as necessary so it is best to test for non zero.

         hw.optional.floatingpoint - Floating Point Instructions
         hw.optional.altivec       - AltiVec Instructions
         hw.optional.graphicsops   - Graphics Operations
         hw.optional.64bitops      - 64-bit Instructions
         hw.optional.fsqrt         - HW Floating Point Square Root Instruction
         hw.optional.stfiwx        - Store Floating Point as Integer Word Indexed Instructions
         hw.optional.dcba          - Data Cache Block Allocate Instruction
         hw.optional.datastreams   - Data Streams Instructions
         hw.optional.dcbtstreams   - Data Cache Block Touch Steams Instruction Form

      }
    {
       CTL_USER definitions
      }
    { string: _CS_PATH  }
    { int: BC_BASE_MAX  }
       USER_BC_BASE_MAX = 2;
    { int: BC_DIM_MAX  }
       USER_BC_DIM_MAX = 3;
    { int: BC_SCALE_MAX  }
       USER_BC_SCALE_MAX = 4;
    { int: BC_STRING_MAX  }
       USER_BC_STRING_MAX = 5;
    { int: COLL_WEIGHTS_MAX  }
       USER_COLL_WEIGHTS_MAX = 6;
    { int: EXPR_NEST_MAX  }
       USER_EXPR_NEST_MAX = 7;
    { int: LINE_MAX  }
       USER_LINE_MAX = 8;
    { int: RE_DUP_MAX  }
       USER_RE_DUP_MAX = 9;
    { int: POSIX2_VERSION  }
       USER_POSIX2_VERSION = 10;
    { int: POSIX2_C_BIND  }
       USER_POSIX2_C_BIND = 11;
    { int: POSIX2_C_DEV  }
       USER_POSIX2_C_DEV = 12;
    { int: POSIX2_CHAR_TERM  }
       USER_POSIX2_CHAR_TERM = 13;
    { int: POSIX2_FORT_DEV  }
       USER_POSIX2_FORT_DEV = 14;
    { int: POSIX2_FORT_RUN  }
       USER_POSIX2_FORT_RUN = 15;
    { int: POSIX2_LOCALEDEF  }
       USER_POSIX2_LOCALEDEF = 16;
    { int: POSIX2_SW_DEV  }
       USER_POSIX2_SW_DEV = 17;
    { int: POSIX2_UPE  }
       USER_POSIX2_UPE = 18;
    { int: POSIX2_STREAM_MAX  }
       USER_STREAM_MAX = 19;
    { int: POSIX2_TZNAME_MAX  }
       USER_TZNAME_MAX = 20;
    { number of valid user ids  }
       USER_MAXID = 21;

