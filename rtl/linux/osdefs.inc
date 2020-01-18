{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 Marco van de Voort
    member of the Free Pascal development team.

    Target dependent defines used when compileing the baseunix unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ifndef cpux86_64}
  {$define usestime}              // Use stime() syscall instead of settimeofday
{$endif}
{$define oldreaddir}            // Keep using readdir system call instead
                                // of userland getdents stuff.
{$define usedomain}             // Allow uname with "domain" entry.
                                // (which is a GNU extension)
{$ifdef FPC_USE_LIBC}
  {$define usegetcwd}
  {$if not defined(cpux86) and not defined(cpux86_64)}
  {$linklib m}
  {$endif}
{$endif}

{$if defined(cpupowerpc) or defined(cpupowerpc64) or defined(cpui386) or
  (defined(cpuarm) and defined(FPC_ABI_EABI))}
  {$DEFINE has_ugetrlimit}
{$endif}

{$if (defined(cpuarm) and defined(FPC_ABI_EABI))}
  {$define FPC_USEGETTIMEOFDAY}
  {$undef usestime}

  {$define NO_SYSCALL_TIME}
  {$define NO_SYSCALL_UMOUNT}
  {$define NO_SYSCALL_STIME}
  {$define NO_SYSCALL_ALARM}
  {$define NO_SYSCALL_UTIME}
  {$define NO_SYSCALL_GETRLIMIT}
  {$define NO_SYSCALL_SELECT}
  {$define NO_SYSCALL_READDIR}
  {$define NO_SYSCALL_MMAP}
  {$define NO_SYSCALL_SOCKETCALL}
  {$define NO_SYSCALL_SYSCALL}
  {$define NO_SYSCALL_IPC}
{$endif}

{$ifdef CPUARM}
{$define WAIT4}
{$endif CPUARM}

{$ifdef CPUx86_64}
{$define WAIT4}
{$endif CPUx86_64}

{$ifdef CPUSPARC}
{$define WAIT4}
{$endif CPUSPARC}

{$ifdef cpui386}
{$define OLDMMAP}
{$endif cpui386}

{$ifdef cpum68k}
{$define OLDMMAP}
{$endif cpum68k}

{$ifdef cpuarm}
  {$ifdef FPC_ABI_EABI}
    {$define MMAP2}
  {$else FPC_ABI_EABI}
    {$define OLDMMAP}
  {$endif FPC_ABI_EABI}
{$endif cpuarm}

{$ifdef cpuaarch64}
  {$define generic_linux_syscalls}
  {$undef usestime}
  {$define userenameat}
{$endif cpuaarch64}

{$ifdef cpusparc64}
  {$define FPC_USEGETTIMEOFDAY}
{$endif cpusparc64}

{$ifdef cpuriscv32}
  {$define generic_linux_syscalls}
  {$undef usestime}
{$endif cpuriscv32}

{$ifdef cpuriscv64}
  {$define generic_linux_syscalls}
  {$undef usestime}
{$endif cpuriscv64}

{$ifdef android}
  {$define generic_linux_syscalls}
  {$ifdef cpuarm}
    {$define NO_SYSCALL_PSELECT6}
    {$define NO_SYSCALL_PPOLL}
  {$endif cpuarm}
  {$define userenameat}
  {$undef usestime}
  {$undef OLDMMAP}
  {$ifndef cpu64}
    {$define MMAP2}
  {$endif}
{$endif android}
