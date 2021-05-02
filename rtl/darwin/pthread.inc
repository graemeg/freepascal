{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Peter Vreman
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This file contains a pthread.h headerconversion,
    and should contain an interface to the threading library to be
    used by systhrd, preferably in a somewhat compatible notation
    (compared to the other OSes).

    As a start, I simply used libc_r

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

CONST PTHREAD_EXPLICIT_SCHED       = 0;
      PTHREAD_CREATE_DETACHED      = 1;
      PTHREAD_SCOPE_PROCESS        = 0;
      SEM_FAILED                   = -1;

 TYPE
    ppthread_t           = ^pthread_t;
    ppthread_key_t       = ^pthread_key_t;
    ppthread_mutex_t     = ^pthread_mutex_t;
    ppthread_attr_t      = ^pthread_attr_t;
    __destr_func_t       = procedure (p :pointer);cdecl;
    __startroutine_t     = function (p :pointer):pointer;cdecl;
    ppthread_mutexattr_t = ^pthread_mutexattr_t;
    ppthread_cond_t      = ^pthread_cond_t;
    ppthread_condattr_t  = ^pthread_condattr_t;

    sem_t       = cint;
    psem_t          = ^sem_t;
    TSemaphore  = sem_t;
    PSemaphore  = ^TSemaphore;

function  pthread_getspecific      (t : pthread_key_t):pointer; cdecl; external 'c';
function  pthread_setspecific      (t : pthread_key_t;p:pointer):cint; cdecl; external 'c';
function  pthread_key_create       (p : ppthread_key_t;f: __destr_func_t):cint; cdecl;external 'c';
function  pthread_attr_init           (p : ppthread_attr_t):cint; cdecl; external 'c';
function  pthread_attr_setinheritsched(p : ppthread_attr_t;i:cint):cint; cdecl; external 'c';
function  pthread_attr_setscope      (p : ppthread_attr_t;i:cint):cint;cdecl;external 'c';
function  pthread_attr_setdetachstate (p : ppthread_attr_t;i:cint):cint;cdecl;external 'c';
function  pthread_attr_setstacksize(p: ppthread_attr_t; stacksize: size_t):cint;cdecl;external 'c';
function  pthread_attr_destroy     (p:ppthread_attr_t):cint;cdecl;external 'c';
function  pthread_create ( p: ppthread_t;attr : ppthread_attr_t;f:__startroutine_t;arg:pointer):cint;cdecl;external 'c';
procedure pthread_exit  ( p: pointer); cdecl;external 'c';
function  pthread_self:pthread_t; cdecl;external 'c';
function  pthread_mutex_init (p:ppthread_mutex_t;o:ppthread_mutexattr_t):cint; cdecl;external 'c';
function  pthread_mutex_destroy (p:ppthread_mutex_t):cint; cdecl;external 'c';
function  pthread_mutex_lock    (p:ppthread_mutex_t):cint; cdecl;external 'c';
function  pthread_mutex_trylock (p:ppthread_mutex_t):cint; cdecl;external 'c';
function  pthread_mutex_unlock  (p:ppthread_mutex_t):cint; cdecl;external 'c';
function  pthread_cancel(_para1:pthread_t):cint;cdecl;external 'c';
function  pthread_detach(_para1:pthread_t):cint;cdecl;external 'c';
function  pthread_join(_para1:pthread_t; _para2:Ppointer):cint;cdecl;external 'c';
function  pthread_cond_destroy(_para1:Ppthread_cond_t):cint;cdecl;external 'c' name 'pthread_cond_destroy';
function  pthread_cond_init(_para1:Ppthread_cond_t;_para2:Ppthread_condattr_t):cint;cdecl;external  'c' name 'pthread_cond_init';
function  pthread_cond_signal(_para1:Ppthread_cond_t):cint;cdecl;external 'c' name 'pthread_cond_signal';
function  pthread_cond_broadcast(_para1:Ppthread_cond_t):cint;cdecl;external 'c' name 'pthread_cond_broadcast';
function  pthread_cond_wait(_para1:Ppthread_cond_t;_para2:Ppthread_mutex_t):cint;cdecl;external 'c' name 'pthread_cond_wait';
function pthread_kill(__thread:pthread_t; __signo:cint):cint;cdecl;external 'c';
function pthread_sigmask(how: cint; nset: psigset; oset: psigset): cint; cdecl; external 'c';

// not yet implemented in Mac OS X 10.4.8!
// function sem_init(__sem:Psem_t; __pshared:cint;__value:cuint):cint;cdecl; external 'c' name 'sem_init';
function sem_open(name: pchar; oflag: cint): Psem_t; cdecl; varargs; external 'c' name 'sem_open';
function sem_destroy(__sem:Psem_t):cint;cdecl;external 'c' name 'sem_destroy';
function sem_close(__sem:Psem_t):cint;cdecl;external 'c'  name 'sem_close';
function sem_unlink(__name:Pchar):cint;cdecl;external 'c' name 'sem_unlink';
function sem_wait(__sem:Psem_t):cint;cdecl;external 'c'  name 'sem_wait';
function sem_trywait(__sem:Psem_t):cint;cdecl;external 'c'  name 'sem_trywait';
function sem_post(__sem:Psem_t):cint;cdecl;external 'c'  name 'sem_post';
function sem_getvalue(__sem:Psem_t; __sval:Pcint):cint;cdecl;external 'c'  name 'sem_getvalue';

function pthread_mutexattr_init(_para1:Ppthread_mutexattr_t):cint;cdecl;external 'c' name 'pthread_mutexattr_init';
function pthread_mutexattr_destroy(_para1:Ppthread_mutexattr_t):cint;cdecl;external 'c' name 'pthread_mutexattr_destroy';
function pthread_mutexattr_gettype(_para1:Ppthread_mutexattr_t; _para2:Pcint):cint;cdecl;external 'c' name 'pthread_mutexattr_gettype';
function pthread_mutexattr_settype(_para1:Ppthread_mutexattr_t; _para2:cint):cint;cdecl;external 'c' name 'pthread_mutexattr_settype';
function pthread_cond_timedwait(__cond:ppthread_cond_t; __mutex:ppthread_mutex_t; __abstime:ptimespec):cint; cdecl;external 'c' name 'pthread_cond_timedwait';

var
  // available in macOS 10.6 / iOS 3.2 and higher
  pthread_setname_np: function(name: PAnsiChar):cint;cdecl;

var
  PthreadDLL: Pointer;

function LoadPthreads: Boolean;
begin
  PThreadDLL:=DlOpen('libSystem.dylib',RTLD_LAZY);
  Result:=PThreadDLL<>Nil;
  if not Result then
    exit;
  Pointer(pthread_setname_np):=dlsym(PthreadDLL,'pthread_setname_np');
end;

function UnLoadPthreads: Boolean;
begin
  Result:=dlclose(PThreadDLL)=0;
end;
