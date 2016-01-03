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

{
 * @file apr_thread_proc.h
 * @brief APR Thread and Process Library
 }

{#include "apr.h"
#include "apr_file_io.h"
#include "apr_pools.h"
#include "apr_errno.h"

#if APR_HAVE_STRUCT_RLIMIT
#include <sys/time.h>
#include <sys/resource.h>
#endif}

{
 * @defgroup apr_thread_proc Threads and Process Functions
 * @ingroup APR 
 }

type
  apr_cmdtype_e = (
    APR_SHELLCMD,       {< use the shell to invoke the program }
    APR_PROGRAM,        {< invoke the program directly, no copied env }
    APR_PROGRAM_ENV,    {< invoke the program, replicating our environment }
    APR_PROGRAM_PATH,   {< find program on PATH, use our environment }
    APR_SHELLCMD_ENV    {< use the shell to invoke the program,
                          *   replicating our environment      }
  );

  apr_wait_how_e = (
    APR_WAIT,           {< wait for the specified process to finish }
    APR_NOWAIT          {< do not wait -- just see if it has finished }
  );

{ I am specifically calling out the values so that the macros below make
 * more sense.  Yes, I know I don't need to, but I am hoping this makes what
 * I am doing more clear.  If you want to add more reasons to exit, continue
 * to use bitmasks.
 }
  apr_exit_why_e = (
    APR_PROC_EXIT = 1,          {< process exited normally }
    APR_PROC_SIGNAL = 2,        {< process exited due to a signal }
    APR_PROC_SIGNAL_CORE = 4    {< process exited and dumped a core file }
  );
  Papr_exit_why_e = ^apr_exit_why_e;

{ did we exit the process }
//#define APR_PROC_CHECK_EXIT(x)        (x & APR_PROC_EXIT)
{ did we get a signal }
//#define APR_PROC_CHECK_SIGNALED(x)    (x & APR_PROC_SIGNAL)
{ did we get core }
//#define APR_PROC_CHECK_CORE_DUMP(x)   (x & APR_PROC_SIGNAL_CORE)

const
{ @see apr_procattr_io_set }
  APR_NO_PIPE         = 0;

{ @see apr_procattr_io_set }
  APR_FULL_BLOCK      = 1;
{ @see apr_procattr_io_set }
  APR_FULL_NONBLOCK   = 2;
{ @see apr_procattr_io_set }
  APR_PARENT_BLOCK    = 3;
{ @see apr_procattr_io_set }
  APR_CHILD_BLOCK     = 4;

{ @see apr_procattr_limit_set }
  APR_LIMIT_CPU       = 0;
{ @see apr_procattr_limit_set }
  APR_LIMIT_MEM       = 1;
{ @see apr_procattr_limit_set }
  APR_LIMIT_NPROC     = 2;
{ @see apr_procattr_limit_set }
  APR_LIMIT_NOFILE    = 3;

{
 * @defgroup APR_OC Other Child Flags
 }
  APR_OC_REASON_DEATH        = 0;     {< child has died, caller must call
                                           * unregister still }
  APR_OC_REASON_UNWRITABLE   = 1;     {< write_fd is unwritable }
  APR_OC_REASON_RESTART      = 2;     {< a restart is occuring, perform
                                           * any necessary cleanup (including
                                           * sending a special signal to child)
                                           }
  APR_OC_REASON_UNREGISTER   = 3;     {< unregister has been called, do
                                           * whatever is necessary (including
                                           * kill the child) }
  APR_OC_REASON_LOST         = 4;     {< somehow the child exited without
                                           * us knowing ... buggy os? }
  APR_OC_REASON_RUNNING      = 5;     {< a health check is occuring,
                                           * for most maintainence functions
                                           * this is a no-op.
                                           }

{ The APR process type }
type
  apr_proc_t = record
    { The process ID }
    pid: pid_t;
    { Parent's side of pipe to child's stdin }
    in_: Papr_file_t;
    { Parent's side of pipe to child's stdout }
    out_: Papr_file_t;
    { Parent's side of pipe to child's stdouterr }
    err: Papr_file_t;
{$if defined(APR_HAS_PROC_INVOKED) or defined(DOXYGEN)}
    { Diagnositics/debugging string of the command invoked for
     *  this process [only present if APR_HAS_PROC_INVOKED is true]
     * @remark Only enabled on Win32 by default.
     * @bug This should either always or never be present in release
     * builds - since it breaks binary compatibility.  We may enable
     * it always in APR 1.0 yet leave it undefined in most cases.
     }
    invoked: PChar;
{$endif}
{$if defined(WINDOWS) or defined(DOXYGEN)}
    { (Win32 only) Creator's handle granting access to the process
     * @remark This handle is closed and reset to NULL in every case
     * corresponding to a waitpid() on Unix which returns the exit status.
     * Therefore Win32 correspond's to Unix's zombie reaping characteristics
     * and avoids potential handle leaks.
     }
    hproc: HANDLE;
{$endif}
  end;
  
  Papr_proc_t = ^apr_proc_t;

{
 * The prototype for APR child errfn functions.  (See the description
 * of apr_procattr_child_errfn_set() for more information.)
 * It is passed the following parameters:
 * @param pool Pool associated with the apr_proc_t.  If your child
 *             error function needs user data, associate it with this
 *             pool.
 * @param err APR error code describing the error
 * @param description Text description of type of processing which failed
 }
  apr_child_errfn_t = procedure (proc: Papr_pool_t;
   err: apr_status_t; const description: PChar);

{ Opaque Thread structure. }
  apr_thread_t = record end;
  Papr_thread_t = ^apr_thread_t;
  PPapr_thread_t = ^Papr_thread_t;

{ Opaque Thread attributes structure. }
  apr_threadattr_t = record end;
  Papr_threadattr_t = ^apr_threadattr_t;
  PPapr_threadattr_t = ^Papr_threadattr_t;

{ Opaque Process attributes structure. }
  apr_procattr_t = record end;
  Papr_procattr_t = ^apr_procattr_t;
  PPapr_procattr_t = ^Papr_procattr_t;

{ Opaque control variable for one-time atomic variables.  }
  apr_thread_once_t = record end;
  Papr_thread_once_t = ^apr_thread_once_t;
  PPapr_thread_once_t = ^Papr_thread_once_t;

{ Opaque thread private address space. }
  apr_threadkey_t = record end;
  Papr_threadkey_t = ^apr_threadkey_t;
  PPapr_threadkey_t = ^Papr_threadkey_t;

{ Opaque record of child process. }
  apr_other_child_rec_t = record end;
  Papr_other_child_rec_t = ^apr_other_child_rec_t;
  PPapr_other_child_rec_t = ^Papr_other_child_rec_t;

{
 * The prototype for any APR thread worker functions.
 }
  apr_thread_start_t = function(param: Papr_thread_t; param2: Pointer): Pointer;

  apr_kill_conditions_e = (
    APR_KILL_NEVER,             {< process is never sent any signals }
    APR_KILL_ALWAYS,            {< process is sent SIGKILL on apr_pool_t cleanup }
    APR_KILL_AFTER_TIMEOUT,     {< SIGTERM, wait 3 seconds, SIGKILL }
    APR_JUST_WAIT,              {< wait forever for the process to complete }
    APR_KILL_ONLY_ONCE          {< send SIGTERM and then wait }
  );

{ Thread Function definitions }

//{$if APR_HAS_THREADS}

{
 * Create and initialize a new threadattr variable
 * @param new_attr The newly created threadattr.
 * @param cont The pool to use
 }
function apr_threadattr_create(new_attr: PPapr_threadattr_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadattr_create' + LibSuff8;

{
 * Set if newly created threads should be created in detached state.
 * @param attr The threadattr to affect 
 * @param on Thread detach state on or off
 }
function apr_threadattr_detach_set(attr: Papr_threadattr_t; on_: apr_int32_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadattr_detach_set' + LibSuff8;

{
 * Get the detach state for this threadattr.
 * @param attr The threadattr to reference 
 }
function apr_threadattr_detach_get(attr: PPapr_threadattr_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadattr_detach_get' + LibSuff4;

{
 * Set the stack size of newly created threads.
 * @param attr The threadattr to affect 
 * @param stacksize The stack size in bytes
 }
function apr_threadattr_stacksize_set(new_attr: PPapr_threadattr_t; stacksize: apr_size_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadattr_stacksize_set' + LibSuff8;

{
 * Create a new thread of execution
 * @param new_thread The newly created thread handle.
 * @param attr The threadattr to use to determine how to create the thread
 * @param func The function to start the new thread in
 * @param data Any data to be passed to the starting function
 * @param cont The pool to use
 }
function apr_thread_create(new_thread: PPapr_thread_t;
 attr: Papr_threadattr_t; func: apr_thread_start_t;
 data: Pointer; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_create' + LibSuff20;

{
 * stop the current thread
 * @param thd The thread to stop
 * @param retval The return value to pass back to any thread that cares
 }
function apr_thread_exit(thd: Papr_thread_t; retval: apr_status_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_exit' + LibSuff8;

{
 * block until the desired thread stops executing.
 * @param retval The return value from the dead thread.
 * @param thd The thread to join
 }
function apr_thread_join(retval: Papr_status_t; thd: Papr_thread_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_join' + LibSuff8;

{
 * force the current thread to yield the processor
 }
procedure apr_thread_yield;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_yield' + LibSuff0;

{
 * Initialize the control variable for apr_thread_once.  If this isn't
 * called, apr_initialize won't work.
 * @param control The control variable to initialize
 * @param p The pool to allocate data from.
 }
function apr_thread_once_init(control: PPapr_thread_once_t; p: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_once_init' + LibSuff8;

{
 * Run the specified function one time, regardless of how many threads
 * call it.
 * @param control The control variable.  The same variable should
 *                be passed in each time the function is tried to be
 *                called.  This is how the underlying functions determine
 *                if the function has ever been called before.
 * @param func The function to call.
 }
type
  func_t = procedure;
  
function apr_thread_once(control: Papr_thread_once_t; func: func_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_once' + LibSuff8;

{
 * detach a thread
 * @param thd The thread to detach 
 }
function apr_thread_detach(thd: Papr_thread_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_detach' + LibSuff4;

{
 * Return the pool associated with the current thread.
 * @param data The user data associated with the thread.
 * @param key The key to associate with the data
 * @param thread The currently open thread.
 }
function apr_thread_data_get(data: PPointer; const key: PChar;
 thread: Papr_thread_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_data_get' + LibSuff12;

{
 * Return the pool associated with the current thread.
 * @param data The user data to associate with the thread.
 * @param key The key to use for associating the data with the thread
 * @param cleanup The cleanup routine to use when the thread is destroyed.
 * @param thread The currently open thread.
 }
function apr_thread_data_set(data: Pointer; const key: PChar;
 cleanup: cleanup_t; thread: Papr_thread_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_thread_data_set' + LibSuff16;

{
 * Create and initialize a new thread private address space
 * @param key The thread private handle.
 * @param dest The destructor to use when freeing the private memory.
 * @param cont The pool to use
 }
type
  dest_t = procedure (param: Pointer);
  
function apr_threadkey_private_create(key: PPapr_threadkey_t;
 dest: dest_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadkey_private_create' + LibSuff12;

{
 * Get a pointer to the thread private memory
 * @param new_mem The data stored in private memory 
 * @param key The handle for the desired thread private memory 
 }
function apr_threadkey_private_get(new_mem: PPointer; key: Papr_threadkey_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadkey_private_get' + LibSuff8;

{
 * Set the data to be stored in thread private memory
 * @param priv The data to be stored in private memory 
 * @param key The handle for the desired thread private memory 
 }
function apr_threadkey_private_set(priv: Pointer; key: Papr_threadkey_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadkey_private_set' + LibSuff8;

{
 * Free the thread private memory
 * @param key The handle for the desired thread private memory 
 }
function apr_threadkey_private_delete(key: Papr_threadkey_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadkey_private_delete' + LibSuff4;

{
 * Return the pool associated with the current threadkey.
 * @param data The user data associated with the threadkey.
 * @param key The key associated with the data
 * @param threadkey The currently open threadkey.
 }
function apr_threadkey_data_get(data: PPointer; const key: PChar;
 threadkey: Papr_threadkey_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadkey_data_get' + LibSuff12;

{
 * Return the pool associated with the current threadkey.
 * @param data The data to set.
 * @param key The key to associate with the data.
 * @param cleanup The cleanup routine to use when the file is destroyed.
 * @param threadkey The currently open threadkey.
 }
function apr_threadkey_data_set(data: Pointer; const key: PChar;
 cleanup: cleanup_t; threadkey: Papr_threadkey_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_threadkey_data_set' + LibSuff16;

{.$endif}

{
 * Create and initialize a new procattr variable
 * @param new_attr The newly created procattr. 
 * @param cont The pool to use
 }
function apr_procattr_create(new_attr: PPapr_procattr_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_create' + LibSuff8;

{
 * Determine if any of stdin, stdout, or stderr should be linked to pipes 
 * when starting a child process.
 * @param attr The procattr we care about. 
 * @param in Should stdin be a pipe back to the parent?
 * @param out Should stdout be a pipe back to the parent?
 * @param err Should stderr be a pipe back to the parent?
 }
function apr_procattr_io_set(attr: Papr_procattr_t;
 in_, out_, err: apr_int32_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_io_set' + LibSuff16;

{
 * Set the child_in and/or parent_in values to existing apr_file_t values.
 * @param attr The procattr we care about. 
 * @param child_in apr_file_t value to use as child_in. Must be a valid file.
 * @param parent_in apr_file_t value to use as parent_in. Must be a valid file.
 * @remark  This is NOT a required initializer function. This is
 *          useful if you have already opened a pipe (or multiple files)
 *          that you wish to use, perhaps persistently across multiple
 *          process invocations - such as a log file. You can save some 
 *          extra function calls by not creating your own pipe since this
 *          creates one in the process space for you.
 }
function apr_procattr_child_in_set(attr: Papr_procattr_t;
 child_in, parent_in: Papr_file_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_child_in_set' + LibSuff12;

{
 * Set the child_out and parent_out values to existing apr_file_t values.
 * @param attr The procattr we care about. 
 * @param child_out apr_file_t value to use as child_out. Must be a valid file.
 * @param parent_out apr_file_t value to use as parent_out. Must be a valid file.
 * @remark This is NOT a required initializer function. This is
 *         useful if you have already opened a pipe (or multiple files)
 *         that you wish to use, perhaps persistently across multiple
 *         process invocations - such as a log file. 
 }
function apr_procattr_child_out_set(attr: Papr_procattr_t;
 child_out, parent_out: Papr_file_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_child_out_set' + LibSuff12;

{
 * Set the child_err and parent_err values to existing apr_file_t values.
 * @param attr The procattr we care about. 
 * @param child_err apr_file_t value to use as child_err. Must be a valid file.
 * @param parent_err apr_file_t value to use as parent_err. Must be a valid file.
 * @remark This is NOT a required initializer function. This is
 *         useful if you have already opened a pipe (or multiple files)
 *         that you wish to use, perhaps persistently across multiple
 *         process invocations - such as a log file. 
 }
function apr_procattr_child_err_set(attr: Papr_procattr_t;
 child_err, parent_err: Papr_file_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_child_err_set' + LibSuff12;

{
 * Set which directory the child process should start executing in.
 * @param attr The procattr we care about. 
 * @param dir Which dir to start in.  By default, this is the same dir as
 *            the parent currently resides in, when the createprocess call
 *            is made. 
 }
function apr_procattr_dir_set(attr: Papr_procattr_t; const dir: PChar): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_dir_set' + LibSuff8;

{
 * Set what type of command the child process will call.
 * @param attr The procattr we care about. 
 * @param cmd The type of command.  One of:
 * <PRE>
 *            APR_SHELLCMD     --  Anything that the shell can handle
 *            APR_PROGRAM      --  Executable program   (default) 
 *            APR_PROGRAM_ENV  --  Executable program, copy environment
 *            APR_PROGRAM_PATH --  Executable program on PATH, copy env
 * </PRE>
 }
function apr_procattr_cmdtype_set(attr: Papr_procattr_t; cmd: apr_cmdtype_e): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_cmdtype_set' + LibSuff8;

{
 * Determine if the child should start in detached state.
 * @param attr The procattr we care about. 
 * @param detach Should the child start in detached state?  Default is no. 
 }
function apr_procattr_detach_set(attr: Papr_procattr_t; detach: apr_int32_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_detach_set' + LibSuff8;

{$ifdef APR_HAVE_STRUCT_RLIMIT}
{
 * Set the Resource Utilization limits when starting a new process.
 * @param attr The procattr we care about. 
 * @param what Which limit to set, one of:
 * <PRE>
 *                 APR_LIMIT_CPU
 *                 APR_LIMIT_MEM
 *                 APR_LIMIT_NPROC
 *                 APR_LIMIT_NOFILE
 * </PRE>
 * @param limit Value to set the limit to.
 }
function apr_procattr_limit_set(attr: Papr_procattr_t; what: apr_int32_t;
 limit: Pointer): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_limit_set' + LibSuff12;

{$endif}

{
 * Specify an error function to be called in the child process if APR
 * encounters an error in the child prior to running the specified program.
 * @param attr The procattr describing the child process to be created.
 * @param errfn The function to call in the child process.
 * @remark At the present time, it will only be called from apr_proc_create()
 *         on platforms where fork() is used.  It will never be called on other
 *         platforms, on those platforms apr_proc_create() will return the error
 *         in the parent process rather than invoke the callback in the now-forked
 *         child process.
 }
function apr_procattr_child_errfn_set(attr: Papr_procattr_t; errfn: apr_child_errfn_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_child_errfn_set' + LibSuff8;

{
 * Specify that apr_proc_create() should do whatever it can to report
 * failures to the caller of apr_proc_create(), rather than find out in
 * the child.
 * @param attr The procattr describing the child process to be created.
 * @param chk Flag to indicate whether or not extra work should be done
 *            to try to report failures to the caller.
 * @remark This flag only affects apr_proc_create() on platforms where
 *         fork() is used.  This leads to extra overhead in the calling
 *         process, but that may help the application handle such
 *         errors more gracefully.
 }
function apr_procattr_error_check_set(attr: Papr_procattr_t; chk: apr_int32_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_error_check_set' + LibSuff8;

{
 * Determine if the child should start in its own address space or using the 
 * current one from its parent
 * @param attr The procattr we care about. 
 * @param addrspace Should the child start in its own address space?  Default
 *                  is no on NetWare and yes on other platforms.
 }
function apr_procattr_addrspace_set(attr: Papr_procattr_t; addrspace: apr_int32_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_procattr_addrspace_set' + LibSuff8;

{$ifdef APR_HAS_FORK}
{
 * This is currently the only non-portable call in APR.  This executes 
 * a standard unix fork.
 * @param proc The resulting process handle. 
 * @param cont The pool to use. 
 }
function apr_proc_fork(proc: Papr_proc_t; cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_fork' + LibSuff8;

{$endif}

{
 * Create a new process and execute a new program within that process.
 * @param new_proc The resulting process handle.
 * @param progname The program to run 
 * @param args the arguments to pass to the new program.  The first 
 *             one should be the program name.
 * @param env The new environment table for the new process.  This 
 *            should be a list of NULL-terminated strings. This argument
 *            is ignored for APR_PROGRAM_ENV, APR_PROGRAM_PATH, and
 *            APR_SHELLCMD_ENV types of commands.
 * @param attr the procattr we should use to determine how to create the new
 *         process
 * @param cont The pool to use. 
 }
function apr_proc_create(new_proc: Papr_proc_t;
 const progname: PChar; args, arnv: PPChar;
 attr: Papr_procattr_t;
 cont: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_create' + LibSuff24;

{
 * Wait for a child process to die
 * @param proc The process handle that corresponds to the desired child process 
 * @param exitcode The returned exit status of the child, if a child process 
 *                 dies, or the signal that caused the child to die.
 *                 On platforms that don't support obtaining this information, 
 *                 the status parameter will be returned as APR_ENOTIMPL.
 * @param exitwhy Why the child died, the bitwise or of:
 * <PRE>
 *            APR_PROC_EXIT         -- process terminated normally
 *            APR_PROC_SIGNAL       -- process was killed by a signal
 *            APR_PROC_SIGNAL_CORE  -- process was killed by a signal, and
 *                                     generated a core dump.
 * </PRE>
 * @param waithow How should we wait.  One of:
 * <PRE>
 *            APR_WAIT   -- block until the child process dies.
 *            APR_NOWAIT -- return immediately regardless of if the 
 *                          child is dead or not.
 * </PRE>
 * @remark The childs status is in the return code to this process.  It is one of:
 * <PRE>
 *            APR_CHILD_DONE     -- child is no longer running.
 *            APR_CHILD_NOTDONE  -- child is still running.
 * </PRE>
 }
function apr_proc_wait(proc: Papr_proc_t; exitcode: PInteger;
 exitwhy: Papr_exit_why_e; waithow: apr_wait_how_e): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_wait' + LibSuff16;

{
 * Wait for any current child process to die and return information 
 * about that child.
 * @param proc Pointer to NULL on entry, will be filled out with child's 
 *             information 
 * @param exitcode The returned exit status of the child, if a child process 
 *                 dies, or the signal that caused the child to die.
 *                 On platforms that don't support obtaining this information, 
 *                 the status parameter will be returned as APR_ENOTIMPL.
 * @param exitwhy Why the child died, the bitwise or of:
 * <PRE>
 *            APR_PROC_EXIT         -- process terminated normally
 *            APR_PROC_SIGNAL       -- process was killed by a signal
 *            APR_PROC_SIGNAL_CORE  -- process was killed by a signal, and
 *                                     generated a core dump.
 * </PRE>
 * @param waithow How should we wait.  One of:
 * <PRE>
 *            APR_WAIT   -- block until the child process dies.
 *            APR_NOWAIT -- return immediately regardless of if the 
 *                          child is dead or not.
 * </PRE>
 * @param p Pool to allocate child information out of.
 * @bug Passing proc as a *proc rather than **proc was an odd choice
 * for some platforms... this should be revisited in 1.0
 }
function apr_proc_wait_all_procs(proc: Papr_proc_t; exitcode: PInteger;
 exitwhy: Papr_exit_why_e; waithow: apr_wait_how_e;
 p: Papr_pool_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_wait_all_procs' + LibSuff20;

const
  APR_PROC_DETACH_FOREGROUND = 0;    {< Do not detach }
  APR_PROC_DETACH_DAEMONIZE = 1;     {< Detach }

{
 * Detach the process from the controlling terminal.
 * @param daemonize set to non-zero if the process should daemonize
 *                  and become a background process, else it will
 *                  stay in the foreground.
 }
{ Not present on the dll }
//function apr_proc_detach(daemonize: Integer): apr_status_t;
// {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
// external LibAPR name LibNamePrefix + 'apr_proc_detach' + LibSuff4;

{
 * Register an other_child -- a child associated to its registered 
 * maintence callback.  This callback is invoked when the process
 * dies, is disconnected or disappears.
 * @param proc The child process to register.
 * @param maintenance maintenance is a function that is invoked with a 
 *                    reason and the data pointer passed here.
 * @param data Opaque context data passed to the maintenance function.
 * @param write_fd An fd that is probed for writing.  If it is ever unwritable
 *                 then the maintenance is invoked with reason 
 *                 OC_REASON_UNWRITABLE.
 * @param p The pool to use for allocating memory.
 * @bug write_fd duplicates the proc->out stream, it's really redundant
 * and should be replaced in the APR 1.0 API with a bitflag of which
 * proc->in/out/err handles should be health checked.
 * @bug no platform currently tests the pipes health.
 }
type
  maintenance_t = procedure (reason: Integer; param: Pointer; status: Integer);
  
procedure apr_proc_other_child_register(proc: Papr_proc_t;
 maintenance: maintenance_t; data: Pointer; write_fd: Papr_file_t;
 p: Papr_pool_t);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_other_child_register' + LibSuff20;

{
 * Stop watching the specified other child.  
 * @param data The data to pass to the maintenance function.  This is
 *             used to find the process to unregister.
 * @warning Since this can be called by a maintenance function while we're
 *          scanning the other_children list, all scanners should protect 
 *          themself by loading ocr->next before calling any maintenance 
 *          function.
 }
procedure apr_proc_other_child_unregister(data: Pointer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_other_child_unregister' + LibSuff4;

{
 * Notify the maintenance callback of a registered other child process
 * that application has detected an event, such as death.
 * @param proc The process to check
 * @param reason The reason code to pass to the maintenance function
 * @param status The status to pass to the maintenance function
 * @remark An example of code using this behavior;
 * <pre>
 * rv = apr_proc_wait_all_procs(&proc, &exitcode, &status, APR_WAIT, p);
 * if (APR_STATUS_IS_CHILD_DONE(rv)) (
 * #if APR_HAS_OTHER_CHILD
 *     if (apr_proc_other_child_alert(&proc, APR_OC_REASON_DEATH, status)
 *             == APR_SUCCESS) (
 *         ;  (already handled)
 *     )
 *     else
 * #endif
 *         [... handling non-otherchild processes death ...]
 * </pre>
 }
function apr_proc_other_child_alert(proc: Papr_proc_t;
 reason, status: Integer): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_other_child_alert' + LibSuff12;

{
 * Test one specific other child processes and invoke the maintenance callback 
 * with the appropriate reason code, if still running, or the appropriate reason 
 * code if the process is no longer healthy.
 * @param ocr The registered other child
 * @param reason The reason code (e.g. APR_OC_REASON_RESTART) if still running
 }
procedure apr_proc_other_child_refresh(ocr: Papr_other_child_rec_t; reason: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_other_child_refresh' + LibSuff8;

{
 * Test all registered other child processes and invoke the maintenance callback 
 * with the appropriate reason code, if still running, or the appropriate reason 
 * code if the process is no longer healthy.
 * @param reason The reason code (e.g. APR_OC_REASON_RESTART) to running processes
 }
procedure apr_proc_other_child_refresh_all(reason: Integer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_other_child_refresh_all' + LibSuff4;

{ @deprecated @see apr_proc_other_child_refresh_all
 * @remark Call apr_proc_other_child_refresh_all(APR_OC_REASON_RESTART)
 * or apr_proc_other_child_refresh_all(APR_OC_REASON_RUNNING) instead.
 * @bug The differing implementations of this function on Win32 (_RUNNING checks) 
 * and Unix (used only for _RESTART) are the reason it will be dropped with APR 1.0.
 }
procedure apr_proc_other_child_check;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_other_child_check' + LibSuff0;

{ @deprecated @see apr_proc_other_child_alert
 * @bug This function's name had nothing to do with it's purpose
 }
function apr_proc_other_child_read(proc: Papr_proc_t; status: Integer): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_other_child_read' + LibSuff8;

{
 * Terminate a process.
 * @param proc The process to terminate.
 * @param sig How to kill the process.
 }
function apr_proc_kill(proc: Papr_proc_t; sig: Integer): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_proc_kill' + LibSuff8;

{
 * Register a process to be killed when a pool dies.
 * @param a The pool to use to define the processes lifetime 
 * @param proc The process to register
 * @param how How to kill the process, one of:
 * <PRE>
 *         APR_KILL_NEVER         -- process is never sent any signals
 *         APR_KILL_ALWAYS        -- process is sent SIGKILL on apr_pool_t cleanup
 *         APR_KILL_AFTER_TIMEOUT -- SIGTERM, wait 3 seconds, SIGKILL
 *         APR_JUST_WAIT          -- wait forever for the process to complete
 *         APR_KILL_ONLY_ONCE     -- send SIGTERM and then wait
 * </PRE>
 }
procedure apr_pool_note_subprocess(a: Papr_pool_t;
 proc: Papr_proc_t; how: apr_kill_conditions_e);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_pool_note_subprocess' + LibSuff12;

{.$ifdef APR_HAS_THREADS}

{$if (defined(APR_HAVE_SIGWAIT) or defined(APR_HAVE_SIGSUSPEND)) and not defined(OS2)}

{
 * Setup the process for a single thread to be used for all signal handling.
 * @warning This must be called before any threads are created
 }
function apr_setup_signal_thread: apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_setup_signal_thread' + LibSuff0;

{
 * Make the current thread listen for signals.  This thread will loop
 * forever, calling a provided function whenever it receives a signal.  That
 * functions should return 1 if the signal has been handled, 0 otherwise.
 * @param signal_handler The function to call when a signal is received
 * apr_status_t apr_signal_thread((int)( *signal_handler)(int signum))
 }
type
  signal_handler_t = function (signum: Integer): Integer;
  
function apr_signal_thread(signal_handler: signal_handler_t): apr_status_t;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF}
 external LibAPR name LibNamePrefix + 'apr_signal_thread' + LibSuff8;

{$endif} { (APR_HAVE_SIGWAIT || APR_HAVE_SIGSUSPEND) && !defined(OS2) }

{
 * Get the child-pool used by the thread from the thread info.
 * @return apr_pool_t the pool
 }
//APR_POOL_DECLARE_ACCESSOR(thread);

//#endif { APR_HAS_THREADS }

