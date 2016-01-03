{ Copyright 2000-2005 The Apache Software Foundation or its licensors, as
 * applicable.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 }

{ This header file is where you should put ANY platform specific information.
 * This should be the only header file that programs need to include that 
 * actually has platform dependant code which refers to the .
 }

{
 * @file apr_portable.h
 * @brief APR Portability Routines
 }

{#include "apr.h"
#include "apr_pools.h"
#include "apr_thread_proc.h"
#include "apr_file_io.h"
#include "apr_network_io.h"
#include "apr_errno.h"
#include "apr_global_mutex.h"
#include "apr_proc_mutex.h"
#include "apr_time.h"
#include "apr_dso.h"
#include "apr_shm.h"

#if APR_HAVE_DIRENT_H
#include <dirent.h>
#endif
#if APR_HAVE_FCNTL_H
#include <fcntl.h>
#endif
#if APR_HAVE_PTHREAD_H
#include <pthread.h>
#endif}

{
 * @defgroup apr_portabile Portability Routines
 * @ingroup APR 
 }



type

{$ifdef WINDOWS}
{ The primitives for Windows types }
  apr_os_file_t = THandle;
  apr_os_dir_t = THandle;
  apr_os_sock_t = TSocket;
  apr_os_proc_mutex_t = THandle;
  apr_os_thread_t = THandle;
  apr_os_proc_t = THandle;
  apr_os_threadkey_t = Cardinal;
  apr_os_imp_time_t = TFILETIME;
  apr_os_exp_time_t = TSYSTEMTIME;
  apr_os_dso_handle_t = THandle;
  apr_os_shm_t = THandle;

{$else}
{$ifdef OS2}
  HFILE                 apr_os_file_t;
  HDIR                  apr_os_dir_t;
  int                   apr_os_sock_t;
  HMTX                  apr_os_proc_mutex_t;
  TID                   apr_os_thread_t;
  PID                   apr_os_proc_t;
  PULONG                apr_os_threadkey_t;
  struct timeval        apr_os_imp_time_t;
  struct tm             apr_os_exp_time_t;
  HMODULE               apr_os_dso_handle_t;
  apr_os_shm_t: Pointer;

{$else}
{$ifdef BEOS}

#include <kernel/OS.h>
#include <kernel/image.h>

  apr_os_proc_mutex_t = record
    sem_id sem;
    int32  ben;
  end;

  apr_os_file_t: cint;
  DIR                   apr_os_dir_t;
  apr_os_sock_t: cint;
  struct apr_os_proc_mutex_t  apr_os_proc_mutex_t;
  thread_id             apr_os_thread_t;
  thread_id             apr_os_proc_t;
  apr_os_threadkey_t: cint;
  struct timeval        apr_os_imp_time_t;
  struct tm             apr_os_exp_time_t;
  image_id              apr_os_dso_handle_t;
  apr_os_shm_t: Pointer;

{$else}
{$ifdef NETWARE}
  apr_os_file_t: cint;
  DIR                   apr_os_dir_t;
  apr_os_sock_t: cint;
  NXMutex_t             apr_os_proc_mutex_t;
  NXThreadId_t          apr_os_thread_t;
  long                  apr_os_proc_t;
  NXKey_t               apr_os_threadkey_t;
  struct timeval        apr_os_imp_time_t;
  struct tm             apr_os_exp_time_t;
  apr_os_dso_handle_t: Pointer;
  apr_os_shm_t: Pointer;

{$else}
{ Any other OS should go above this one.  This is the lowest common
 * denominator typedefs for  all UNIX-like systems.  :)
 }

{ Basic OS process mutex structure. }
  apr_os_proc_mutex_t = record
{$if defined(APR_HAS_SYSVSEM_SERIALIZE) or defined(APR_HAS_FCNTL_SERIALIZE) or defined(APR_HAS_FLOCK_SERIALIZE)}
    crossproc: Integer;
{$endif}
{$ifdef APR_HAS_PROC_PTHREAD_SERIALIZE}
    pthread_mutex_t *pthread_interproc;
{$endif}
{$ifdef APR_HAS_THREADS}
    { If no threads, no need for thread locks }
{$if APR_USE_PTHREAD_SERIALIZE}
    pthread_mutex_t *intraproc;
{$endif}
{$endif}
  end;

  apr_os_file_t: Integer;        {< native file }
typedef DIR                   apr_os_dir_t;         {< native dir }
  apr_os_sock_t: Integer;        {< native dir }
typedef struct apr_os_proc_mutex_t  apr_os_proc_mutex_t; {< native proces
                                                          *   mutex
                                                          }
{$if defined(APR_HAS_THREADS) and defined(APR_HAVE_PTHREAD_H)}
typedef pthread_t             apr_os_thread_t;      {< native thread }
typedef pthread_key_t         apr_os_threadkey_t;   {< native thread address
                                                     *   space }
{$endif}
typedef pid_t                 apr_os_proc_t;        {< native pid }
typedef struct timeval        apr_os_imp_time_t;    {< native timeval }
typedef struct tm             apr_os_exp_time_t;    {< native tm }
{ @var apr_os_dso_handle_t
 * native dso types
 }
{$ifdef HPUX}
#include <dl.h>
typedef shl_t                 apr_os_dso_handle_t;
{$else}
{$ifdef DARWIN}
#include <mach-o/dyld.h>
typedef NSModule              apr_os_dso_handle_t;
{$else}
typedef void *                apr_os_dso_handle_t;
{$endif}
{$endif}
typedef void*                 apr_os_shm_t;         {< native SHM }

{$endif}
{$endif}
{$endif}
{$endif}

  Papr_os_sock_t = ^apr_os_sock_t;
  
{
 * @typedef apr_os_sock_info_t
 * @brief alias for local OS socket
 }
{
 * everything APR needs to know about an active socket to construct
 * an APR socket from it; currently, this is platform-independent
 }
  apr_os_sock_info_t = record
    os_sock: Papr_os_sock_t; {< always required }
    local: Psockaddr; {< NULL if not yet bound }
    remote: Psockaddr; {< NULL if not connected }
    family: Integer;             {< always required (APR_INET, APR_INET6, etc.) }
    type_: Integer;               {< always required (SOCK_STREAM, SOCK_DGRAM, etc.) }
    protocol: Integer;           {< 0 or actual protocol (APR_PROTO_SCTP, APR_PROTO_TCP, etc.) }
  end;

  Papr_os_sock_info_t = ^apr_os_sock_info_t;

{$if defined(APR_PROC_MUTEX_IS_GLOBAL) or defined(DOXYGEN)}
{ Opaque global mutex type }
#define apr_os_global_mutex_t apr_os_proc_mutex_t
{ @return apr_os_global_mutex }
#define apr_os_global_mutex_get apr_os_proc_mutex_get
{$else}
    { Thread and process mutex for those platforms where process mutexes
     *  are not held in threads.
     }
    apr_os_global_mutex_t = record
      pool: Papr_pool_t;
      proc_mutex: Papr_proc_mutex_t;
#if APR_HAS_THREADS
      thread_mutex: Papr_proc_mutex_t;
#endif { APR_HAS_THREADS }
    end;
    Papr_os_global_mutex_t = ^apr_os_global_mutex_t;

APR_DECLARE(apr_status_t) apr_os_global_mutex_get(apr_os_global_mutex_t *ospmutex, 
                                                apr_global_mutex_t *pmutex);
{$endif}


{
 * convert the file from apr type to os specific type.
 * @param thefile The os specific file we are converting to
 * @param file The apr file to convert.
 * @remark On Unix, it is only possible to get a file descriptor from 
 *         an apr file type.
 }
function apr_os_file_get(thefile, file_: Papr_os_file_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_file_get' + LibSuff8;

{
 * convert the dir from apr type to os specific type.
 * @param thedir The os specific dir we are converting to
 * @param dir The apr dir to convert.
 }
function apr_os_dir_get(thedir: PPapr_os_dir_t; dir: Papr_dir_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_dir_get' + LibSuff8;

{
 * Convert the socket from an apr type to an OS specific socket
 * @param thesock The socket to convert.
 * @param sock The os specifc equivelant of the apr socket..
 }
function apr_os_sock_get(thesock, sock: Papr_os_sock_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_sock_get' + LibSuff8;

{
 * Convert the proc mutex from os specific type to apr type
 * @param ospmutex The os specific proc mutex we are converting to.
 * @param pmutex The apr proc mutex to convert.
 }
function apr_os_proc_mutex_get(ospmutex: Papr_os_proc_mutex_t;
 pmutex: Papr_proc_mutex_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_proc_mutex_get' + LibSuff8;

{
 * Get the exploded time in the platforms native format.
 * @param ostime the native time format
 * @param aprtime the time to convert
 }
function apr_os_exp_time_get(ostime: PPapr_os_exp_time_t;
 aprtime: Papr_time_exp_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_exp_time_get' + LibSuff8;

{
 * Get the imploded time in the platforms native format.
 * @param ostime  the native time format
 * @param aprtime the time to convert
 }
function apr_os_imp_time_get(ostime: PPapr_os_imp_time_t;
 aprtime: Papr_time_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_imp_time_get' + LibSuff8;

{
 * convert the shm from apr type to os specific type.
 * @param osshm The os specific shm representation
 * @param shm The apr shm to convert.
 }
function apr_os_shm_get(osshm: Papr_os_shm_t; shm: Papr_shm_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_shm_get' + LibSuff8;

{$if defined(APR_HAS_THREADS) or defined(DOXYGEN)}
{
 * @defgroup apr_os_thread Thread portability Routines
 }
{
 * convert the thread to os specific type from apr type.
 * @param thethd The apr thread to convert
 * @param thd The os specific thread we are converting to
 }
function apr_os_thread_get(thethd: PPapr_os_thread_t;
 thd: Papr_thread_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_thread_get' + LibSuff8;

{
 * convert the thread private memory key to os specific type from an apr type.
 * @param thekey The apr handle we are converting from.
 * @param key The os specific handle we are converting to.
 }
function apr_os_threadkey_get(thekey: Papr_os_threadkey_t;
 key: Papr_threadkey_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_threadkey_get' + LibSuff8;

{
 * convert the thread from os specific type to apr type.
 * @param thd The apr thread we are converting to.
 * @param thethd The os specific thread to convert
 * @param cont The pool to use if it is needed.
 }
function apr_os_thread_put(thd: PPapr_thread_t;
 thethd: Papr_os_thread_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_thread_put' + LibSuff12;

{
 * convert the thread private memory key from os specific type to apr type.
 * @param key The apr handle we are converting to.
 * @param thekey The os specific handle to convert
 * @param cont The pool to use if it is needed.
 }
function apr_os_threadkey_put(key: PPapr_threadkey_t;
 thekey: Papr_os_threadkey_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_threadkey_put' + LibSuff12;

{
 * Get the thread ID
 }
function apr_os_thread_current: apr_os_thread_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_thread_current' + LibSuff0;

{
 * Compare two thread id's
 * @param tid1 1st Thread ID to compare
 * @param tid2 2nd Thread ID to compare
 }
function apr_os_thread_equal(tid1, tid2: apr_os_thread_t): Integer;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_thread_equal' + LibSuff8;

{$endif} { APR_HAS_THREADS }

{
 * convert the file from os specific type to apr type.
 * @param file The apr file we are converting to.
 * @param thefile The os specific file to convert
 * @param flags The flags that were used to open this file.
 * @param cont The pool to use if it is needed.
 * @remark On Unix, it is only possible to put a file descriptor into
 *         an apr file type.
 }
function apr_os_file_put(file_: PPapr_file_t;
 pthefilemutex: Papr_os_file_t; flags: apr_int32_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_file_put' + LibSuff16;

{
 * convert the file from os specific type to apr type.
 * @param file The apr file we are converting to.
 * @param thefile The os specific pipe to convert
 * @param cont The pool to use if it is needed.
 * @remark On Unix, it is only possible to put a file descriptor into
 *         an apr file type.
 }
function apr_os_pipe_put(file_: PPapr_file_t;
 thefile: Papr_os_file_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_pipe_put' + LibSuff12;

{
 * convert the file from os specific type to apr type.
 * @param file The apr file we are converting to.
 * @param thefile The os specific pipe to convert
 * @param register_cleanup A cleanup will be registered on the apr_file_t
 *   to issue apr_file_close().
 * @param cont The pool to use if it is needed.
 * @remark On Unix, it is only possible to put a file descriptor into
 *         an apr file type.
 }
function apr_os_pipe_put_ex(file_: PPapr_file_t;
 thefile: Papr_os_file_t; register_cleanup: Integer;
 cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_pipe_put_ex' + LibSuff16;

{
 * convert the dir from os specific type to apr type.
 * @param dir The apr dir we are converting to.
 * @param thedir The os specific dir to convert
 * @param cont The pool to use when creating to apr directory.
 }
function apr_os_dir_put(dir: PPapr_dir_t; thedir: Papr_os_dir_t;
 cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_dir_put' + LibSuff12;

{
 * Convert a socket from the os specific type to the apr type
 * @param sock The pool to use.
 * @param thesock The socket to convert to.
 * @param cont The socket we are converting to an apr type.
 * @remark If it is a true socket, it is best to call apr_os_sock_make()
 *         and provide APR with more information about the socket.
 }
function apr_os_sock_put(sock: PPapr_socket_t; thesock: Papr_socket_t;
 cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_sock_put' + LibSuff8;

{
 * Create a socket from an existing descriptor and local and remote
 * socket addresses.
 * @param apr_sock The new socket that has been set up
 * @param os_sock_info The os representation of the socket handle and
 *        other characteristics of the socket
 * @param cont The pool to use
 * @remark If you only know the descriptor/handle or if it isn't really
 *         a true socket, use apr_os_sock_put() instead.
 }
function apr_os_sock_make(apr_sock: PPapr_socket_t;
 os_sock_info: Papr_os_sock_info_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_sock_make' + LibSuff12;

{
 * Convert the proc mutex from os specific type to apr type
 * @param pmutex The apr proc mutex we are converting to.
 * @param ospmutex The os specific proc mutex to convert.
 * @param cont The pool to use if it is needed.
 }
function apr_os_proc_mutex_put(pmutex: PPapr_proc_mutex_t;
 ospmutex: Papr_os_proc_mutex_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_proc_mutex_put' + LibSuff12;

{
 * Put the imploded time in the APR format.
 * @param aprtime the APR time format
 * @param ostime the time to convert
 * @param cont the pool to use if necessary
 }
function apr_os_imp_time_put(aprtime: Papr_time_t;
 ostime: PPapr_os_imp_time_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_imp_time_put' + LibSuff12;

{
 * Put the exploded time in the APR format.
 * @param aprtime the APR time format
 * @param ostime the time to convert
 * @param cont the pool to use if necessary
 }
function apr_os_exp_time_put(aprtime: Papr_time_exp_t;
 ostime: PPapr_os_exp_time_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_exp_time_put' + LibSuff12;

{
 * convert the shared memory from os specific type to apr type.
 * @param shm The apr shm representation of osshm
 * @param osshm The os specific shm identity
 * @param cont The pool to use if it is needed.
 * @remark On fork()ed architectures, this is typically nothing more than
 * the memory block mapped.  On non-fork architectures, this is typically
 * some internal handle to pass the mapping from process to process.
 }
function apr_os_shm_put(shm: PPapr_shm_t; osshm: Papr_os_shm_t;
 cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_shm_put' + LibSuff8;

{$if defined(APR_HAS_DSO) or defined(DOXYGEN)}
{
 * @defgroup apr_os_dso DSO (Dynamic Loading) Portabiliity Routines
 }
{
 * convert the dso handle from os specific to apr
 * @param dso The apr handle we are converting to
 * @param thedso the os specific handle to convert
 * @param pool the pool to use if it is needed
 }
function apr_os_dso_handle_put(dso: PPapr_dso_handle_t;
 thedso: apr_os_dso_handle_t; pool: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_dso_handle_put' + LibSuff12;

{
 * convert the apr dso handle into an os specific one
 * @param aprdso The apr dso handle to convert
 * @param dso The os specific dso to return
 }
function apr_os_dso_handle_get(dso: Papr_os_dso_handle_t;
 aprdso: Papr_dso_handle_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_dso_handle_get' + LibSuff8;

{$ifdef APR_HAS_OS_UUID}
{
 * Private: apr-util's apr_uuid module when supported by the platform
 }
function apr_os_uuid_get(uuid_data: PChar): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_uuid_get' + LibSuff8;
{$endif}

{$endif} { APR_HAS_DSO }


{
 * Get the name of the system default characer set.
 * @param pool the pool to allocate the name from, if needed
 }
function apr_os_default_encoding(pool: Papr_pool_t): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_default_encoding' + LibSuff4;

{
 * Get the name of the current locale character set.
 * @param pool the pool to allocate the name from, if needed
 * @remark Defers to apr_os_default_encoding if the current locale's
 * data can't be retreved on this system.
 }
function apr_os_locale_encoding(pool: Papr_pool_t): PChar;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_os_locale_encoding' + LibSuff4;

