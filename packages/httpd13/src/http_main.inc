{ Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
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
 * Routines in http_main.c which other code --- in particular modules ---
 * may want to call.  Right now, that's limited to timeout handling.
 * There are two functions which modules can call to trigger a timeout
 * (with the per-virtual-server timeout duration); these are hard_timeout
 * and soft_timeout.
 *
 * The difference between the two is what happens when the timeout
 * expires (or earlier than that, if the client connection aborts) ---
 * a soft_timeout just puts the connection to the client in an
 * "aborted" state, which will cause http_protocol.c to stop trying to
 * talk to the client, but otherwise allows the code to continue normally.
 * hard_timeout(), by contrast, logs the request, and then aborts it
 * completely --- longjmp()ing out to the accept() loop in http_main.
 * Any resources tied into the request's resource pool will be cleaned up;
 * everything that isn't will leak.
 *
 * soft_timeout() is recommended as a general rule, because it gives your
 * code a chance to clean up.  However, hard_timeout() may be the most
 * convenient way of dealing with timeouts waiting for some external
 * resource other than the client, if you can live with the restrictions.
 *
 * (When a hard timeout is in scope, critical sections can be guarded
 * with block_alarms() and unblock_alarms() --- these are declared in
 * alloc.c because they are most often used in conjunction with
 * routines to allocate something or other, to make sure that the
 * cleanup does get registered before any alarm is allowed to happen
 * which might require it to be cleaned up; they * are, however,
 * implemented in http_main.c).
 *
 * NOTE!  It's not "fair" for a hard_timeout to be in scope through calls
 * across modules.  Your module code really has no idea what other modules may
 * be present in the server, and they may not take too kindly to having a
 * longjmp() happen -- it could result in corrupted state.  Heck they may not
 * even take to kindly to a soft_timeout()... because it can cause EINTR to
 * happen on pretty much any syscall, and unless all the libraries and modules
 * in use are known to deal well with EINTR it could cause corruption as well.
 * But things are likely to do much better with a soft_timeout in scope than a
 * hard_timeout.
 * 
 * A module MAY NOT use a hard_timeout() across * sub_req_lookup_xxx()
 * functions, or across run_sub_request() functions.  A module SHOULD NOT use a
 * soft_timeout() in either of these cases, but sometimes there's just no
 * choice.
 *
 * kill_timeout() will disarm either variety of timeout.
 *
 * reset_timeout() resets the timeout in progress.
 }

procedure ap_start_shutdown();
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

procedure ap_start_restart(param: cint);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

procedure ap_hard_timeout(p: PChar; r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

procedure ap_keepalive_timeout(p: PChar; r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

procedure ap_soft_timeout(p: PChar; r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

procedure ap_kill_timeout(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

procedure ap_reset_timeout(r: Prequest_rec);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;


//procedure ap_child_terminate(r: Prequest_rec);
// {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

procedure ap_sync_scoreboard_image();
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

function ap_update_child_status(child_num, status: cint; r: Prequest_rec): cint;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

{ void ap_time_process_request(int child_num, int status); }

type
  fn_t = procedure (param: cint);

function ap_set_callback_and_alarm(fn: fn_t; x: cint): cuint;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

function ap_check_alarm(): cint;
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

{ void setup_signal_names(char *prefix);}

{ functions for determination and setting of accept() mutexing }
{char *ap_default_mutex_method(void);
char *ap_init_mutex_method(char *t);}

{$ifndef NO_OTHER_CHILD}
{
 * register an other_child -- a child which the main loop keeps track of
 * and knows it is different than the rest of the scoreboard.
 *
 * pid is the pid of the child.
 *
 * maintenance is a function that is invoked with a reason, the data
 * pointer passed here, and when appropriate a status result from waitpid().
 *
 * write_fd is an fd that is probed for writing by select() if it is ever
 * unwritable, then maintenance is invoked with reason OC_REASON_UNWRITABLE.
 * This is useful for log pipe children, to know when they've blocked.  To
 * disable this feature, use -1 for write_fd.
 }
type
  maintenance_t = procedure (reason: cint; data: Pointer; status: ap_wait_t);
  
procedure ap_register_other_child(pid: cint;
 maintenance: maintenance_t; data: Pointer; write_fd: cint);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

const
  OC_REASON_DEATH      = 0;	{ child has died, caller must call
					 * unregister still }
  OC_REASON_UNWRITABLE = 1;	{ write_fd is unwritable }
  OC_REASON_RESTART    = 2;	{ a restart is occuring, perform
					 * any necessary cleanup (including
					 * sending a special signal to child)
                                         }
  OC_REASON_UNREGISTER = 3;	{ unregister has been called, do
					 * whatever is necessary (including
					 * kill the child) }
  OC_REASON_LOST       = 4;	{ somehow the child exited without
					 * us knowing ... buggy os? }

{
 * unregister an other_child.  Note that the data pointer is used here, and
 * is assumed to be unique' per other_child.  This is because the pid and
 * write_fd are possibly killed off separately.
 }
procedure ap_unregister_other_child(data: Pointer);
 {$IFDEF WINDOWS} stdcall; {$ELSE} cdecl; {$ENDIF} external LibHTTPD;

{$endif}

