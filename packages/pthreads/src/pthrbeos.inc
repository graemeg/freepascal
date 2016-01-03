{
   This file is part of the Free Pascal run time library.
   Copyright (c) 1999-2000 by Michael Van Canneyt,
   BSD parts (c) 2000 by Marco van de Voort
   members of the Free Pascal development team.

   New linux unit. Linux only calls only. Will be renamed to linux.pp
   when 1.0.x support is killed off.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

{
  Automatically converted by H2Pas 0.99.15 from pthrwrk2.h
}

{$linklib pthread}      // The compiler changes this to c_r if needed.
{$linklib c}            // (parameter -Xf)

{$PACKRECORDS C}


  {
     Copyright (c) 1993, 1994 by Chris Provenzano, proven@mit.edu
     Copyright (c) 1995-1998 by John Birrell <jb@cimlogic.com.au>
     All rights reserved.

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
      This product includes software developed by Chris Provenzano.
     4. The name of Chris Provenzano may not be used to endorse or promote
          products derived from this software without specific prior written
          permission.

     THIS SOFTWARE IS PROVIDED BY CHRIS PROVENZANO ``AS IS'' AND
     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     ARE DISCLAIMED.  IN NO EVENT SHALL CHRIS PROVENZANO BE LIABLE FOR ANY
     DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
     OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
     SUCH DAMAGE.

     $FreeBSD: src/include/pthread.h,v 1.20.2.4 2003/05/27 18:18:01 jdp Exp $
    }

    {
     Run-time invariant values:
    }


CONST ULONG_MAX= high(culong);

type

     psize_t   = ^size_t;
     psigset_t = ^sigset_t;

const
    PTHREAD_MUTEX_NORMAL     = _PTHREAD_MUTEX_NORMAL;
    PTHREAD_MUTEX_ERRORCHECK = _PTHREAD_MUTEX_ERRORCHECK;
    PTHREAD_MUTEX_RECURSIVE  = _PTHREAD_MUTEX_RECURSIVE;

     PTHREAD_MUTEX_DEFAULT    = _PTHREAD_MUTEX_NORMAL;
     MUTEX_TYPE_FAST          = _PTHREAD_MUTEX_NORMAL;
     MUTEX_TYPE_COUNTING_FAST = _PTHREAD_MUTEX_RECURSIVE;


     PTHREAD_DESTRUCTOR_ITERATIONS = 4;
     PTHREAD_KEYS_MAX              = _PTHREAD_KEYS_MAX;
     PTHREAD_STACK_MIN             = _PTHREAD_STACK_MIN;
     PTHREAD_THREADS_MAX           = ULONG_MAX;
    {
     Flags for threads and thread attributes.
    }
     PTHREAD_DETACHED              = $1;
     PTHREAD_SCOPE_SYSTEM          = $2;
     PTHREAD_INHERIT_SCHED         = $4;
     PTHREAD_NOFLOAT               = $8;
     PTHREAD_CREATE_DETACHED       = PTHREAD_DETACHED;
     PTHREAD_CREATE_JOINABLE       = 0;
     PTHREAD_SCOPE_PROCESS         = 0;
     PTHREAD_EXPLICIT_SCHED        = 0;

    {
     Flags for read/write lock attributes
    }
     PTHREAD_PROCESS_PRIVATE       = 0;
     PTHREAD_PROCESS_SHARED        = 1;
    {
     Flags for cancelling threads
    }
     PTHREAD_CANCEL_ENABLE         = 0;
     PTHREAD_CANCEL_DISABLE        = 1;
     PTHREAD_CANCEL_DEFERRED       = 0;
     PTHREAD_CANCEL_ASYNCHRONOUS   = 2;
     PTHREAD_CANCELED              = pointer(1);

  {
     Forward structure definitions.

     These are mostly opaque to the user.
    }

  type
{ from semaphore.h }

     psem_t = ^sem_t;
     TSemaphore         = sem_t;
     PSemaphore   = ^TSemaphore;

    {
     Primitive system data type definitions required by P1003.1c.

     Note that P1003.1c specifies that there are no defined comparison
     or assignment operators for the types pthread_attr_t, pthread_cond_t,
     pthread_condattr_t, pthread_mutex_t, pthread_mutexattr_t.
    }

<<<<<<< HEAD
<<<<<<< HEAD
=======
     TThreadID = pthread_t;
     PThreadID = ^TThreadID;
>>>>>>> graemeg/fixes_2_2
=======
     TThreadID = pthread_t;
     PThreadID = ^TThreadID;
>>>>>>> origin/fixes_2_2
     TThreadAttr = pthread_attr_t;
     PThreadAttr = ^TThreadAttr;
     TPthreadMutex = pthread_mutex_t;
     PTpthreadmutex= ^TPthreadMutex;
     TMutexAttribute = pthread_mutexattr_t;
     PMutexAttribute = ^TMutexAttribute;
     TPthreadCondattr = pthread_condattr_t;
     PPthreadCondattr = ^TPthreadCondattr;

{added, adjust these if hdr conv went wrong}
     {from sched.h}

     sched_param =  record
                      sched_priority : cint;
                    end;

     {for pthread.h}
     ppthread_attr_t            = ^pthread_attr_t;
     ppthread_cond_t            = ^pthread_cond_t;
     ppthread_condattr_t        = ^pthread_condattr_t;
     ppthread_mutex_t           = ^pthread_mutex_t;
     ppthread_t                 = ^pthread_t;
     ppthread_key_t             = ^pthread_key_t;
     ppthread_mutexattr_t       = ^pthread_mutexattr_t;
     ppthread_rwlock_t          = ^pthread_rwlock_t;
     ppthread_rwlockattr_t      = ^pthread_rwlockattr_t;
     ppthread_once_t            = ^pthread_once_t;
     psched_param               = ^sched_param;
     TSchedParam = sched_param;
     PSchedParam = ^TSchedParam;

    {
     Additional type definitions:

     Note that P1003.1c reserves the prefixes pthread_ and PTHREAD_ for
     use in header symbols.
    }

     pthread_addr_t             = packed record end;
     pthread_startroutine_t     = procedure; cdecl;

    {
     Once definitions.
    }
     _pthread_once = record
          state : cint;
          mutex : pthread_mutex_t;
       end;

     pthread_once_t             = _pthread_once;

    {
     Flags for once initialization.
    }

  const
     PTHREAD_NEEDS_INIT         = 0;
     PTHREAD_DONE_INIT          = 1;
  {
     Static once initialization values.
  }

  { ????
#define PTHREAD_ONCE_INIT   ( PTHREAD_NEEDS_INIT, NULL )
  }


      {
        Static initialization values.
      }
       PTHREAD_MUTEX_INITIALIZER        = NIL;
       PTHREAD_COND_INITIALIZER         = NIL;
       PTHREAD_RWLOCK_INITIALIZER       = NIL;
    {
       Default attribute arguments (draft 4, deprecated).
    }

{$ifndef PTHREAD_KERNEL}
    const
       pthread_condattr_default         = NIL;
       pthread_mutexattr_default        = NIL;
       pthread_attr_default             = NiL;
{$endif}

    const
       PTHREAD_PRIO_NONE                = 0;
       PTHREAD_PRIO_INHERIT             = 1;
       PTHREAD_PRIO_PROTECT             = 2;

    {
       Thread function prototype definitions:
      }

function pthread_attr_destroy (p : ppthread_attr_t ):cint; cdecl; external;

function pthread_attr_getstack (p : ppthread_attr_t; stackaddr : ppointer;
                        stacksize : psize_t):cint; cdecl; external;

function pthread_attr_getstacksize(_para1:Ppthread_attr_t; _para2:Psize_t):cint;cdecl; external;
function pthread_attr_getstackaddr(_para1:Ppthread_attr_t; _para2:Ppointer):cint;cdecl;external;
function pthread_attr_getdetachstate(_para1:Ppthread_attr_t; _para2:Pcint):cint;cdecl; external;
function pthread_attr_init(_para1:Ppthread_attr_t):cint;cdecl;external;
function pthread_attr_setstack(_para1:Ppthread_attr_t; _para2:pointer; _para3:size_t):cint;cdecl;external;
function pthread_attr_setstacksize(_para1:Ppthread_attr_t; _para2:size_t):cint;cdecl;external;
function pthread_attr_setstackaddr(_para1:Ppthread_attr_t; _para2:pointer):cint;cdecl;external;
function pthread_attr_setdetachstate(_para1:Ppthread_attr_t; _para2:cint):cint;cdecl;external;
procedure pthread_cleanup_pop(_para1:cint);cdecl;external;

type cleanuproutine= function (a:pointer):pointer;cdecl;

procedure pthread_cleanup_push(_para1:cleanuproutine; routine_arg:pointer);cdecl;external;
function pthread_condattr_destroy(_para1:Ppthread_condattr_t):cint;cdecl;external;
function pthread_condattr_init(_para1:Ppthread_condattr_t):cint;cdecl;external;

{$ifdef POSIX_THREAD_PROCESS_SHARED}
function pthread_condattr_getpshared(_para1:Ppthread_condattr_t; _para2:Pcint):cint;cdecl;external;
function pthread_condattr_setpshared(_para1:Ppthread_condattr_t; _para2:cint):cint;cdecl;external;
{$endif}

function pthread_cond_broadcast(_para1:Ppthread_cond_t):cint;cdecl;external;
function pthread_cond_destroy(_para1:Ppthread_cond_t):cint;cdecl;external;
function pthread_cond_init(_para1:Ppthread_cond_t; _para2:Ppthread_condattr_t):cint;cdecl;external;
function pthread_cond_signal(_para1:Ppthread_cond_t):cint;cdecl;external;
function pthread_cond_timedwait(_para1:Ppthread_cond_t; _para2:Ppthread_mutex_t; _para3:Ptimespec):cint;cdecl;external;
function pthread_cond_wait(_para1:Ppthread_cond_t; _para2:Ppthread_mutex_t):cint;cdecl;external;
type pthreadcreateroutine = function (p:pointer):pointer; cdecl;
function pthread_create(_para1:Ppthread_t; _para2:Ppthread_attr_t; _para3:Pthreadcreateroutine; _para4:pointer):cint;cdecl;external;
function pthread_detach(_para1:pthread_t):cint;cdecl;external;
function pthread_equal(_para1:pthread_t; _para2:pthread_t):cint;cdecl;external;
procedure pthread_exit  ( p: pointer); cdecl;external;  { __dead2}
function pthread_getspecific(_para1:pthread_key_t):pointer;cdecl;external;
function pthread_join(_para1:pthread_t; _para2:Ppointer):cint;cdecl;external;
type pthreadkeycreateroutine = function (p:pointer):pointer; cdecl;
function pthread_key_create(_para1:Ppthread_key_t; _para2:pthreadkeycreateroutine):cint;cdecl;external;
function pthread_key_delete(_para1:pthread_key_t):cint;cdecl;external;
function pthread_kill(_para1:pthread_t; _para2:cint):cint;cdecl;external;
function pthread_mutexattr_init(_para1:Ppthread_mutexattr_t):cint;cdecl;external;
function pthread_mutexattr_destroy(_para1:Ppthread_mutexattr_t):cint;cdecl;external;
function pthread_mutexattr_gettype(_para1:Ppthread_mutexattr_t; _para2:Pcint):cint;cdecl;external;
function pthread_mutexattr_settype(_para1:Ppthread_mutexattr_t; _para2:cint):cint;cdecl;external;

function pthread_mutex_destroy(_para1:Ppthread_mutex_t):cint;cdecl;external;
function pthread_mutex_init(_para1:Ppthread_mutex_t; _para2:Ppthread_mutexattr_t):cint;cdecl;external;
function pthread_mutex_lock(_para1:Ppthread_mutex_t):cint;cdecl;external;
function pthread_mutex_trylock(_para1:Ppthread_mutex_t):cint;cdecl;external;
function pthread_mutex_unlock(_para1:Ppthread_mutex_t):cint;cdecl;external;
type pthreadonceroutine = procedure (p:pointer); cdecl;
function pthread_once(_para1:Ppthread_once_t; _para2:pthreadonceroutine ):cint;cdecl;external;
function pthread_rwlock_destroy(_para1:Ppthread_rwlock_t):cint;cdecl;external;
function pthread_rwlock_init(_para1:Ppthread_rwlock_t; _para2:Ppthread_rwlockattr_t):cint;cdecl;external;
function pthread_rwlock_rdlock(_para1:Ppthread_rwlock_t):cint;cdecl;external;
function pthread_rwlock_tryrdlock(_para1:Ppthread_rwlock_t):cint;cdecl;external;
function pthread_rwlock_trywrlock(_para1:Ppthread_rwlock_t):cint;cdecl;external;
function pthread_rwlock_unlock(_para1:Ppthread_rwlock_t):cint;cdecl;external;
function pthread_rwlock_wrlock(_para1:Ppthread_rwlock_t):cint;cdecl;external;
function pthread_rwlockattr_init(_para1:Ppthread_rwlockattr_t):cint;cdecl;external;
function pthread_rwlockattr_getpshared(_para1:Ppthread_rwlockattr_t; _para2:Pcint):cint;cdecl;external;
function pthread_rwlockattr_setpshared(_para1:Ppthread_rwlockattr_t; _para2:cint):cint;cdecl;external;
function pthread_rwlockattr_destroy(_para1:Ppthread_rwlockattr_t):cint;cdecl;external;
function pthread_self:pthread_t;cdecl;external;
function pthread_setspecific(_para1:pthread_key_t; _para2:pointer):cint;cdecl;external;
function pthread_sigmask(_para1:cint; _para2:Psigset_t; _para3:Psigset_t):cint;cdecl;external;
function pthread_cancel(_para1:pthread_t):cint;cdecl;external;
function pthread_setcancelstate(_para1:cint; _para2:Pcint):cint;cdecl;external;
function pthread_setcanceltype(_para1:cint; _para2:Pcint):cint;cdecl;external;
procedure pthread_testcancel;cdecl;external;
function pthread_getprio(_para1:pthread_t):cint;cdecl;external;
function pthread_setprio(_para1:pthread_t; _para2:cint):cint;cdecl;external;
procedure pthread_yield;cdecl;external;

{$ifdef POSIX_THREAD_PROCESS_SHARED}
function pthread_mutexattr_getpshared(_para1:Ppthread_mutexattr_t; pshared:Pcint):cint;cdecl;external;
function pthread_mutexattr_setpshared(_para1:Ppthread_mutexattr_t; pshared:cint):cint;cdecl;external;
{$endif}

function pthread_mutexattr_getprioceiling(_para1:Ppthread_mutexattr_t; _para2:Pcint):cint;cdecl;external;
function pthread_mutexattr_setprioceiling(_para1:Ppthread_mutexattr_t; _para2:cint):cint;cdecl;external;
function pthread_mutex_getprioceiling(_para1:Ppthread_mutex_t; _para2:Pcint):cint;cdecl;external;
function pthread_mutex_setprioceiling(_para1:Ppthread_mutex_t; _para2:cint; _para3:Pcint):cint;cdecl;external;
function pthread_mutexattr_getprotocol(_para1:Ppthread_mutexattr_t; _para2:Pcint):cint;cdecl;external;
function pthread_mutexattr_setprotocol(_para1:Ppthread_mutexattr_t; _para2:cint):cint;cdecl;external;
function pthread_attr_getinheritsched(_para1:Ppthread_attr_t; _para2:Pcint):cint;cdecl;external;
function pthread_attr_getschedparam(_para1:Ppthread_attr_t; _para2:Psched_param):cint;cdecl;external;
function pthread_attr_getschedpolicy(_para1:Ppthread_attr_t; _para2:Pcint):cint;cdecl;external;
function pthread_attr_getscope(_para1:Ppthread_attr_t; _para2:Pcint):cint;cdecl;external;
function pthread_attr_setinheritsched(_para1:Ppthread_attr_t; _para2:cint):cint;cdecl;external;
function pthread_attr_setschedparam(_para1:Ppthread_attr_t; _para2:Psched_param):cint;cdecl;external;
function pthread_attr_setschedpolicy(_para1:Ppthread_attr_t; _para2:cint):cint;cdecl;external;
function pthread_attr_setscope(_para1:Ppthread_attr_t; _para2:cint):cint;cdecl;external;
function pthread_getschedparam(pthread:pthread_t; _para2:Pcint; _para3:Psched_param):cint;cdecl;external;
function pthread_setschedparam(_para1:pthread_t; _para2:cint; _para3:Psched_param):cint;cdecl;external;
function pthread_getconcurrency:cint;cdecl;external;
function pthread_setconcurrency(_para1:cint):cint;cdecl;external;
function pthread_attr_setfloatstate(_para1:Ppthread_attr_t; _para2:cint):cint;cdecl;external;
function pthread_attr_getfloatstate(_para1:Ppthread_attr_t; _para2:Pcint):cint;cdecl;external;

{ semaphore.h }


  function sem_init(__sem:Psem_t; __pshared:cint;__value:dword):cint;cdecl; external;
  function sem_destroy(__sem:Psem_t):cint;cdecl;external ;
  function sem_close(__sem:Psem_t):cint;cdecl;external ;
  function sem_unlink(__name:Pchar):cint;cdecl;external ;
  function sem_wait(__sem:Psem_t):cint;cdecl;external ;
  function sem_trywait(__sem:Psem_t):cint;cdecl;external ;
  function sem_post(__sem:Psem_t):cint;cdecl;external ;
  function sem_getvalue(__sem:Psem_t; __sval:Pcint):cint;cdecl;external;


  function sem_init(var __sem: sem_t; __pshared:cint; __value:dword):cint cdecl;external;
  function sem_destroy(var __sem: sem_t):cint;cdecl;external;
  function sem_close(var __sem: sem_t):cint;cdecl;external;
  function sem_wait(var __sem: sem_t):cint;cdecl;external;
  function sem_trywait(var __sem: sem_t):cint;cdecl;external;
  function sem_post(var __sem: sem_t):cint;cdecl;external;
  function sem_getvalue(var __sem: sem_t; var __sval:cint):cint;cdecl;external;



