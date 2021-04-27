{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit handles the pass_typecheck and node conversion pass

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit pass_1;

{$i fpcdefs.inc}

interface

    uses
       node;

    procedure typecheckpass(var p : tnode);
    function  do_typecheckpass(var p : tnode) : boolean;
    function  do_typecheckpass_changed(var p : tnode; out nodechanged: boolean) : boolean;

    procedure firstpass(var p : tnode);
    function  do_firstpass(var p : tnode) : boolean;
{$ifdef state_tracking}
    procedure  do_track_state_pass(p:Tnode);
{$endif}


implementation

    uses
      globtype,comphook,
      globals,
      procinfo,
      symdef
{$ifdef extdebug}
      ,verbose,htypechk
      ,cgbase
{$endif extdebug}
{$ifdef state_tracking}
      ,nstate
{$endif}
      ;

{*****************************************************************************
                            Global procedures
*****************************************************************************}

    procedure typecheckpass_internal_loop(var p : tnode; out node_changed: boolean);
      var
         hp        : tnode;
         oldflags  : tnodeflags;
      begin
        codegenerror:=false;
        repeat
          current_filepos:=p.fileinfo;
          current_settings.localswitches:=p.localswitches;
          status.verbosity:=p.verbosity;
          hp:=p.pass_typecheck;
          { should the node be replaced? }
          if assigned(hp) then
            begin
              node_changed:=true;
              oldflags:=p.flags;
              p.free;
              { switch to new node }
              p:=hp;
              { transfer generic paramter flag }
              if nf_generic_para in oldflags then
                include(p.flags,nf_generic_para);
            end;
        until not assigned(hp) or
              assigned(hp.resultdef);
        if codegenerror then
          begin
            include(p.flags,nf_error);
            { default to errortype if no type is set yet }
            if p.resultdef=nil then
              p.resultdef:=generrordef;
          end;
      end;

    procedure typecheckpass_internal(var p : tnode; out node_changed: boolean);
      var
         oldcodegenerror  : boolean;
         oldlocalswitches : tlocalswitches;
         oldverbosity     : longint;
         oldpos    : tfileposinfo;
      begin
        node_changed:=false;
        if (p.resultdef=nil) then
          begin
            oldcodegenerror:=codegenerror;
            oldpos:=current_filepos;
            oldlocalswitches:=current_settings.localswitches;
            oldverbosity:=status.verbosity;
            typecheckpass_internal_loop(p, node_changed);
            current_settings.localswitches:=oldlocalswitches;
            current_filepos:=oldpos;
            status.verbosity:=oldverbosity;
            codegenerror:=codegenerror or oldcodegenerror;
          end
        else
          begin
            { update the codegenerror boolean with the previous result of this node }
            if (nf_error in p.flags) then
              codegenerror:=true;
          end;
      end;


    procedure typecheckpass(var p : tnode);
      var
        node_changed: boolean;
      begin
        typecheckpass_internal(p,node_changed);
      end;


    function do_typecheckpass_changed(var p : tnode; out nodechanged: boolean) : boolean;
      begin
         codegenerror:=false;
         typecheckpass_internal(p,nodechanged);
         do_typecheckpass_changed:=codegenerror;
      end;


    function do_typecheckpass(var p : tnode) : boolean;
      var
        nodechanged: boolean;
      begin
         result:=do_typecheckpass_changed(p,nodechanged);
      end;


    procedure firstpass(var p : tnode);
      var
         oldcodegenerror  : boolean;
         oldlocalswitches : tlocalswitches;
         oldpos    : tfileposinfo;
         oldverbosity: longint;
         hp : tnode;
         nodechanged : boolean;
      begin
         if (nf_pass1_done in p.flags) then
           exit;
         if not(nf_error in p.flags) then
           begin
             oldcodegenerror:=codegenerror;
             oldpos:=current_filepos;
             oldlocalswitches:=current_settings.localswitches;
             oldverbosity:=status.verbosity;
             codegenerror:=false;
             repeat
               { checks make always a call }
               if ([cs_check_range,cs_check_overflow,cs_check_stack] * current_settings.localswitches <> []) then
                 include(current_procinfo.flags,pi_do_call);
               { determine the resultdef if not done }
               if (p.resultdef=nil) then
                 begin
                   typecheckpass_internal_loop(p,nodechanged);
                 end;

               hp:=nil;
               if not(nf_error in p.flags) then
                 begin
                   current_filepos:=p.fileinfo;
                   current_settings.localswitches:=p.localswitches;
                   status.verbosity:=p.verbosity;
                   { first pass }
                   hp:=p.pass_1;
                   { inlining happens in pass_1 and can cause new }
                   { simplify opportunities                       }
                   if not assigned(hp) then
                     hp:=p.simplify(true);
                   { should the node be replaced? }
                   if assigned(hp) then
                     begin
                       hp.flags := hp.flags + (p.flags * [nf_usercode_entry]);
                       p.free;
                       { switch to new node }
                       p:=hp;
                     end;
                   if codegenerror then
                     include(p.flags,nf_error);
                 end;
             until not assigned(hp) or
                   (nf_pass1_done in hp.flags);
             include(p.flags,nf_pass1_done);
{$ifdef EXTDEBUG}
             if not(nf_error in p.flags) then
               begin
                 if (p.expectloc=LOC_INVALID) then
                   Comment(V_Warning,'Expectloc is not set in firstpass: '+nodetype2str[p.nodetype]);
               end;
{$endif EXTDEBUG}
             codegenerror:=codegenerror or oldcodegenerror;
             current_settings.localswitches:=oldlocalswitches;
             current_filepos:=oldpos;
             status.verbosity:=oldverbosity;
           end
         else
           codegenerror:=true;
      end;


    function do_firstpass(var p : tnode) : boolean;
      begin
         codegenerror:=false;
         firstpass(p);
{$ifdef state_tracking}
         writeln('TRACKSTART');
         writeln('before');
         writenode(p);
         do_track_state_pass(p);
         writeln('after');
         writenode(p);
         writeln('TRACKDONE');
{$endif}
         do_firstpass:=codegenerror;
      end;

{$ifdef state_tracking}
     procedure do_track_state_pass(p:Tnode);

     begin
        aktstate:=Tstate_storage.create;
        p.track_state_pass(true);
            aktstate.destroy;
     end;
{$endif}

end.
