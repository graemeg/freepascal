{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Information about the current procedure that is being compiled

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
unit procinfo;

{$i fpcdefs.inc}

  interface

    uses
      { common }
      cclasses,
      { global }
      globtype,
      { symtable }
      symconst,symtype,symdef,symsym,
      node,
      { aasm }
      cpubase,cgbase,cgutils,
      aasmbase,aasmdata;

    const
      inherited_inlining_flags : tprocinfoflags =
        [pi_do_call,
         { the stack frame can't be removed in this case }
         pi_has_assembler_block,
         pi_uses_exceptions];


    type
       tsavedlabels = array[Boolean] of TAsmLabel;

       { This object gives information on the current routine being
         compiled.
       }
       tprocinfo = class(tlinkedlistitem)
       private
          { list to store the procinfo's of the nested procedures }
          nestedprocs : tlinkedlist;
          { required alignment for this stackframe }
          fstackalignment : longint;
          procedure addnestedproc(child: tprocinfo);
       public
          { pointer to parent in nested procedures }
          parent : tprocinfo;
          { the definition of the routine itself }
          procdef : tprocdef;
          { nested implicit finalzation procedure, used for platform-specific
            exception handling }
          finalize_procinfo : tprocinfo;
          { file location of begin of procedure }
          entrypos  : tfileposinfo;
          { file location of end of procedure }
          exitpos   : tfileposinfo;
          { local switches at begin of procedure }
          entryswitches : tlocalswitches;
          { local switches at end of procedure }
          exitswitches  : tlocalswitches;

          { Size of the parameters on the stack }
          para_stack_size : pint;

          { Offset of temp after para/local are allocated }
          tempstart : longint;

          { some collected informations about the procedure
            see pi_xxxx constants above
          }
          flags : tprocinfoflags;

          { register used as frame pointer }
          framepointer : tregister;

          { register containing currently the got }
          got : tregister;
          CurrGOTLabel : tasmlabel;

          { register containing the tlsoffset }
          tlsoffset : tregister;
          { reference label for tls addresses }
          tlslabel : tasmlabel;

          { Holds the reference used to store all saved registers. }
          save_regs_ref : treference;

          { Last assembler instruction of procedure prologue }
          endprologue_ai : tlinkedlistitem;

          { Amount of stack adjustment after all alignments }
          final_localsize : longint;

          { Labels for TRUE/FALSE condition, BREAK and CONTINUE }
          CurrBreakLabel,
          CurrContinueLabel : tasmlabel;

          { label to leave the sub routine }
          CurrExitLabel : tasmlabel;

          { label for nested exits }
          nestedexitlabel : tlabelsym;

          { The code for the routine itself, excluding entry and
            exit code. This is a linked list of tai classes.
          }
          aktproccode : TAsmList;
          { Data (like jump tables) that belongs to this routine }
          aktlocaldata : TAsmList;

          { max. of space need for parameters }
          maxpushedparasize : aint;

          { some architectures need to know a stack size before the first compilation pass
            estimatedtempsize contains an estimated value how big temps will get }
          estimatedtempsize : longint;

          { is this a constructor that calls another constructor on itself
            (either inherited, or another constructor of the same class)?
            Requires different entry code for some targets. }
          ConstructorCallingConstructor: boolean;

          { true, if an FPU instruction has been generated which could raise an exception and where the flags
            need to be checked explicitly like on RISC-V or certain ARM architectures }
          FPUExceptionCheckNeeded : Boolean;

          { local symbols and defs referenced by global functions; these need
            to be exported in case the function gets inlined }
          localrefsyms : tfpobjectlist;
          localrefdefs : tfpobjectlist;

          constructor create(aparent:tprocinfo);virtual;
          destructor destroy;override;

          procedure allocate_push_parasize(size:longint);

          function calc_stackframe_size:longint;virtual;abstract;

          { Set the address of the first temp, can be used to allocate
            space for pushing parameters }
          procedure set_first_temp_offset;virtual;

          { Generate parameter information }
          procedure generate_parameter_info;virtual;

          { Allocate got register }
          procedure allocate_got_register(list: TAsmList);virtual;

          { Allocate tls register }
          procedure allocate_tls_register(list: TAsmList);virtual;

          { get frame pointer }
          procedure init_framepointer; virtual;

          { Destroy the entire procinfo tree, starting from the outermost parent }
          procedure destroy_tree;

          function get_first_nestedproc: tprocinfo;
          function has_nestedprocs: boolean;
          function get_normal_proc: tprocinfo;

          procedure add_local_ref_sym(sym:tsym);
          procedure export_local_ref_syms;
          procedure add_local_ref_def(def:tdef);
          procedure export_local_ref_defs;

          function create_for_outlining(const basesymname: string; astruct: tabstractrecorddef; potype: tproctypeoption; resultdef: tdef; entrynodeinfo: tnode): tprocinfo;

          { Add to parent's list of nested procedures even if parent is a 'main' procedure }
          procedure force_nested;

          { Get the required alignment for the current stack frame }
          property stackalignment: longint read fstackalignment;
          { Update the resuired alignment for the current stack frame based
            on the current value and the new required alignment }
          procedure updatestackalignment(alignment: longint);
          { Specific actions after the code has been generated }
          procedure postprocess_code; virtual;

          { set exception handling info }
          procedure set_eh_info; virtual;

          procedure setup_eh; virtual;
          procedure finish_eh; virtual;
          { called to insert needed eh info into the entry code }
          procedure start_eh(list : TAsmList); virtual;
          { called to insert needed eh info into the exit code }
          procedure end_eh(list : TAsmList); virtual;
       end;
       tcprocinfo = class of tprocinfo;

    var
       cprocinfo : tcprocinfo;
       { information about the current sub routine being parsed (@var(pprocinfo))}
       current_procinfo : tprocinfo;

implementation

    uses
      globals,cutils,systems,verbose,
      procdefutil;

{****************************************************************************
                                 TProcInfo
****************************************************************************}

    constructor tprocinfo.create(aparent:tprocinfo);
      begin
        parent:=aparent;
        procdef:=nil;
        para_stack_size:=0;
        fstackalignment:=target_info.stackalign;
        flags:=[];
        init_framepointer;
        framepointer:=NR_FRAME_POINTER_REG;
        maxpushedparasize:=0;
        { asmlists }
        aktproccode:=TAsmList.Create;
        aktlocaldata:=TAsmList.Create;
        reference_reset(save_regs_ref,sizeof(aint),[]);
        { labels }
        current_asmdata.getjumplabel(CurrExitLabel);
        current_asmdata.getjumplabel(CurrGOTLabel);
        CurrBreakLabel:=nil;
        CurrContinueLabel:=nil;
        if Assigned(parent) and (parent.procdef.parast.symtablelevel>=normal_function_level) then
          parent.addnestedproc(Self);
      end;

    procedure tprocinfo.force_nested;
      begin
        if Assigned(parent) and (parent.procdef.parast.symtablelevel<normal_function_level) then
          parent.addnestedproc(Self);
      end;

    destructor tprocinfo.destroy;
      begin
         nestedprocs.free;
         aktproccode.free;
         aktlocaldata.free;
         localrefsyms.free;
         localrefdefs.free;
      end;

    procedure tprocinfo.destroy_tree;
      var
        hp: tprocinfo;
      begin
        hp:=Self;
        while Assigned(hp.parent) do
          hp:=hp.parent;
        hp.Free;
      end;

    procedure tprocinfo.addnestedproc(child: tprocinfo);
      begin
        if nestedprocs=nil then
          nestedprocs:=TLinkedList.Create;
        nestedprocs.insert(child);
      end;

    procedure tprocinfo.updatestackalignment(alignment: longint);
      begin
        fstackalignment:=max(fstackalignment,alignment);
      end;

    function tprocinfo.get_first_nestedproc: tprocinfo;
      begin
        if assigned(nestedprocs) then
          result:=tprocinfo(nestedprocs.first)
        else
          result:=nil;
      end;

    function tprocinfo.has_nestedprocs: boolean;
      begin
        result:=assigned(nestedprocs) and (nestedprocs.count>0);
      end;

    function tprocinfo.get_normal_proc: tprocinfo;
      begin
        result:=self;
        while assigned(result.parent) and (result.procdef.parast.symtablelevel>normal_function_level) do
          result:=result.parent;
      end;

    procedure tprocinfo.add_local_ref_sym(sym:tsym);
      begin
        if not assigned(localrefsyms) then
          localrefsyms:=tfpobjectlist.create(false);
        if localrefsyms.indexof(sym)<0 then
          localrefsyms.add(sym);
      end;

    procedure tprocinfo.export_local_ref_syms;
      var
        i : longint;
        sym : tsym;
      begin
        if not assigned(localrefsyms) then
          exit;
        for i:=0 to localrefsyms.count-1 do
          begin
            sym:=tsym(localrefsyms[i]);
            if sym.typ<>staticvarsym then
              internalerror(2019110901);
            include(tstaticvarsym(sym).varoptions,vo_has_global_ref);
          end;
      end;

    procedure tprocinfo.add_local_ref_def(def:tdef);
      begin
        if not assigned(localrefdefs) then
          localrefdefs:=tfpobjectlist.create(false);
        if localrefdefs.indexof(def)<0 then
          localrefdefs.add(def);
      end;

    procedure tprocinfo.export_local_ref_defs;
      var
        i : longint;
        def : tdef;
      begin
        if not assigned(localrefdefs) then
          exit;
        for i:=0 to localrefdefs.count-1 do
          begin
            def:=tdef(localrefdefs[i]);
            if def.typ<>symconst.procdef then
              internalerror(2019111801);
            include(tprocdef(def).defoptions,df_has_global_ref);
          end;
      end;

    function tprocinfo.create_for_outlining(const basesymname: string; astruct: tabstractrecorddef; potype: tproctypeoption; resultdef: tdef; entrynodeinfo: tnode): tprocinfo;
      begin
        result:=cprocinfo.create(self);
        result.force_nested;
        result.procdef:=create_outline_procdef(basesymname,astruct,potype,resultdef);
        result.entrypos:=entrynodeinfo.fileinfo;
        result.entryswitches:=entrynodeinfo.localswitches;
        result.exitpos:=current_filepos; // filepos of last node?
        result.exitswitches:=current_settings.localswitches; // localswitches of last node?
      end;

    procedure tprocinfo.allocate_push_parasize(size:longint);
      begin
        if size>maxpushedparasize then
          maxpushedparasize:=size;
      end;

    procedure tprocinfo.set_first_temp_offset;
      begin
      end;

    procedure tprocinfo.generate_parameter_info;
      begin
        { generate callee paraloc register info, it initialises the size that
          is allocated on the stack }
        procdef.init_paraloc_info(calleeside);
        para_stack_size:=procdef.calleeargareasize;
      end;

    procedure tprocinfo.allocate_got_register(list: TAsmList);
      begin
        { most os/cpu combo's don't use this yet, so not yet abstract }
      end;

    procedure tprocinfo.allocate_tls_register(list : TAsmList);
      begin
      end;


    procedure tprocinfo.init_framepointer;
      begin
        { most targets use a constant, but some have a typed constant that must
          be initialized }
      end;


    procedure tprocinfo.postprocess_code;
      begin
        { no action by default }
      end;


    procedure tprocinfo.set_eh_info;
      begin
        { default code is in tcgprocinfo }
      end;


    procedure tprocinfo.setup_eh;
      begin
        { no action by default }
      end;


    procedure tprocinfo.finish_eh;
      begin
        { no action by default }
      end;


    procedure tprocinfo.start_eh(list: TAsmList);
      begin
        { no action by default }
      end;


    procedure tprocinfo.end_eh(list: TAsmList);
      begin
        { no action by default }
      end;

end.
