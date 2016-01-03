{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit does the parsing process

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
unit parser;

{$i fpcdefs.inc}

interface

{$ifdef PREPROCWRITE}
    procedure preprocess(const filename:string);
{$endif PREPROCWRITE}
    procedure compile(const filename:string);
    procedure initparser;
    procedure doneparser;

implementation

    uses
{$IFNDEF USE_FAKE_SYSUTILS}
      sysutils,
{$ELSE}
      fksysutl,
{$ENDIF}
      cutils,cclasses,
      globtype,version,tokens,systems,globals,verbose,switches,globstat,
      symbase,symtable,symdef,symsym,
      finput,fmodule,fppu,
      aasmbase,aasmtai,aasmdata,
      cgbase,
      script,gendef,
      comphook,
      scanner,scandir,
      pbase,ptype,psystem,pmodules,psub,ncgrtti,htypechk,
      cresstr,cpuinfo,procinfo;


    procedure initparser;
      begin
         { Current compiled module/proc }
         set_current_module(nil);
         current_module:=nil;
         current_asmdata:=nil;
         current_procinfo:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
         current_structdef:=nil;
         current_genericdef:=nil;
         current_specializedef:=nil;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

         loaded_units:=TLinkedList.Create;

         usedunits:=TLinkedList.Create;

         unloaded_units:=TLinkedList.Create;

         { global switches }
         current_settings.globalswitches:=init_settings.globalswitches;

         current_settings.sourcecodepage:=init_settings.sourcecodepage;

         { initialize scanner }
         InitScanner;
         InitScannerDirectives;

         { scanner }
         c:=#0;
         pattern:='';
         orgpattern:='';
         cstringpattern:='';
         current_scanner:=nil;
         switchesstatestackpos:=0;

         { register all nodes and tais }
         registernodes;
         registertais;

         { memory sizes }
         if stacksize=0 then
           stacksize:=target_info.stacksize;

         { RTTI writer }
         RTTIWriter:=TRTTIWriter.Create;

         { open assembler response }
         if cs_link_on_target in current_settings.globalswitches then
           GenerateAsmRes(outputexedir+ChangeFileExt(inputfilename,'_ppas'))
         else
           GenerateAsmRes(outputexedir+'ppas');

         { open deffile }
         DefFile:=TDefFile.Create(outputexedir+ChangeFileExt(inputfilename,target_info.defext));

         { list of generated .o files, so the linker can remove them }
         SmartLinkOFiles:=TCmdStrList.Create;

         { codegen }
         if paraprintnodetree<>0 then
           printnode_reset;

         { target specific stuff }
         case target_info.system of
           system_powerpc_amiga,
           system_powerpc_morphos,
           system_m68k_amiga,
           system_i386_aros,
           system_x86_64_aros:
             include(supported_calling_conventions,pocall_syscall);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef i8086}
           system_i8086_msdos:
             begin
               if stacksize=0 then
                 begin
                   if init_settings.x86memorymodel in x86_far_data_models then
                     stacksize:=16384
                   else
                     stacksize:=4096;
                 end;
               if maxheapsize=0 then
                 begin
                   if init_settings.x86memorymodel in x86_far_data_models then
                     maxheapsize:=655360
                   else
                     maxheapsize:=65520;
                 end;
             end;
           system_i8086_win16:
             begin
               if stacksize=0 then
                 begin
                   if init_settings.x86memorymodel in x86_far_data_models then
                     stacksize:=8192
                   else
                     stacksize:=5120;
                 end;
               if heapsize=0 then
                 begin
                   if init_settings.x86memorymodel in x86_far_data_models then
                     heapsize:=8192
                   else
                     heapsize:=4096;
                 end;
             end;
{$endif i8086}
=======
>>>>>>> graemeg/fixes_2_2
=======
           system_m68k_amiga:
             include(supported_calling_conventions,pocall_syscall);
>>>>>>> origin/fixes_2_2
         end;
      end;


    procedure doneparser;
      begin
         { Reset current compiling info, so destroy routines can't
           reference the data that might already be destroyed }
         set_current_module(nil);
         current_module:=nil;
         current_procinfo:=nil;
         current_asmdata:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
         current_structdef:=nil;
         current_genericdef:=nil;
         current_specializedef:=nil;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

         { unload units }
         if assigned(loaded_units) then
           begin
             loaded_units.free;
             loaded_units:=nil;
           end;
         if assigned(usedunits) then
           begin
             usedunits.free;
             usedunits:=nil;
           end;
         if assigned(unloaded_units) then
           begin
             unloaded_units.free;
             unloaded_units:=nil;
           end;

         { if there was an error in the scanner, the scanner is
           still assinged }
         if assigned(current_scanner) then
          begin
            current_scanner.free;
            current_scanner:=nil;
          end;

         { close scanner }
         DoneScanner;

         RTTIWriter.free;

         { close ppas,deffile }
         asmres.free;
         deffile.free;

         { free list of .o files }
         SmartLinkOFiles.Free;
      end;




{$ifdef PREPROCWRITE}
    procedure preprocess(const filename:string);
      var
        i : longint;
      begin
         new(preprocfile,init('pre'));
       { initialize a module }
         set_current_module(new(pmodule,init(filename,false)));

         macrosymtablestack:= initialmacrosymtable;
         current_module.localmacrosymtable:= tmacrosymtable.create(false);
         current_module.localmacrosymtable.next:= initialmacrosymtable;
         macrosymtablestack:= current_module.localmacrosymtable;

         main_module:=current_module;
       { startup scanner, and save in current_module }
         current_scanner:=new(pscannerfile,Init(filename));
         current_module.scanner:=current_scanner;
       { loop until EOF is found }
         repeat
           current_scanner^.readtoken(true);
           preprocfile^.AddSpace;
           case token of
             _ID :
               begin
                 preprocfile^.Add(orgpattern);
               end;
             _REALNUMBER,
             _INTCONST :
               preprocfile^.Add(pattern);
             _CSTRING :
               begin
                 i:=0;
                 while (i<length(cstringpattern)) do
                  begin
                    inc(i);
                    if cstringpattern[i]='''' then
                     begin
                       insert('''',cstringpattern,i);
                       inc(i);
                     end;
                  end;
                 preprocfile^.Add(''''+cstringpattern+'''');
               end;
             _CCHAR :
               begin
                 case pattern[1] of
                   #39 :
                     pattern:='''''''';
                   #0..#31,
                   #128..#255 :
                     begin
                       str(ord(pattern[1]),pattern);
                       pattern:='#'+pattern;
                     end;
                   else
                     pattern:=''''+pattern[1]+'''';
                 end;
                 preprocfile^.Add(pattern);
               end;
             _EOF :
               break;
             else
               preprocfile^.Add(tokeninfo^[token].str)
           end;
         until false;
       { free scanner }
         dispose(current_scanner,done);
         current_scanner:=nil;
       { close }
         dispose(preprocfile,done);
      end;
{$endif PREPROCWRITE}


{*****************************************************************************
                             Compile a source file
*****************************************************************************}

    procedure compile(const filename:string);
<<<<<<< HEAD
=======
      type
        polddata=^tolddata;
        tolddata=record
        { scanner }
          oldidtoken,
          oldtoken       : ttoken;
          oldtokenpos    : tfileposinfo;
          oldc           : char;
          oldpattern,
          oldorgpattern  : string;
          old_block_type : tblock_type;
        { symtable }
          oldsymtablestack,
          oldmacrosymtablestack : TSymtablestack;
          oldaktprocsym    : tprocsym;
        { cg }
          oldparse_only  : boolean;
        { akt.. things }
          oldcurrent_filepos      : tfileposinfo;
          old_current_module : tmodule;
          oldcurrent_procinfo : tprocinfo;
          old_settings : tsettings;
          oldsourcecodepage : tcodepagestring;
        end;

>>>>>>> graemeg/fixes_2_2
      var
         olddata : pglobalstate;
         hp,hp2 : tmodule;
         finished : boolean;
       begin
         { parsing a procedure or declaration should be finished }
         if assigned(current_procinfo) then
           internalerror(200811121);
         if assigned(current_structdef) then
           internalerror(200811122);
         inc(compile_level);
         parser_current_file:=filename;
         { Uses heap memory instead of placing everything on the
           stack. This is needed because compile() can be called
           recursively }
         new(olddata);
<<<<<<< HEAD
         { handle the postponed case first }
         flushpendingswitchesstate;
         save_global_state(olddata^,false);

=======
         with olddata^ do
          begin
            old_current_module:=current_module;
          { save symtable state }
            oldsymtablestack:=symtablestack;
            oldmacrosymtablestack:=macrosymtablestack;
            oldcurrent_procinfo:=current_procinfo;
          { save scanner state }
            oldc:=c;
            oldpattern:=pattern;
            oldorgpattern:=orgpattern;
            oldtoken:=token;
            oldidtoken:=idtoken;
            old_block_type:=block_type;
            oldtokenpos:=current_tokenpos;
          { save cg }
            oldparse_only:=parse_only;
          { save akt... state }
          { handle the postponed case first }
           if localswitcheschanged then
             begin
               current_settings.localswitches:=nextlocalswitches;
               localswitcheschanged:=false;
             end;
            oldcurrent_filepos:=current_filepos;
            old_settings:=current_settings;
          end;
>>>>>>> graemeg/fixes_2_2
       { reset parser, a previous fatal error could have left these variables in an unreliable state, this is
         important for the IDE }
         afterassignment:=false;
         in_args:=false;
         named_args_allowed:=false;
         got_addrn:=false;
         getprocvardef:=nil;
         allow_array_constructor:=false;

       { show info }
         Message1(parser_i_compiling,filename);

       { reset symtable }
         symtablestack:=tdefawaresymtablestack.create;
         macrosymtablestack:=TSymtablestack.create;
         systemunit:=nil;
         current_settings.defproccall:=init_settings.defproccall;
         current_exceptblock:=0;
         exceptblockcounter:=0;
         current_settings.maxfpuregisters:=-1;
         current_settings.pmessage:=nil;
       { reset the unit or create a new program }
         { a unit compiled at command line must be inside the loaded_unit list }
         if (compile_level=1) then
           begin
             if assigned(current_module) then
               internalerror(200501158);
<<<<<<< HEAD
<<<<<<< HEAD
             set_current_module(tppumodule.create(nil,'',filename,false));
=======
             set_current_module(tppumodule.create(nil,filename,'',false));
>>>>>>> graemeg/fixes_2_2
=======
             set_current_module(tppumodule.create(nil,filename,'',false));
>>>>>>> origin/fixes_2_2
             addloadedunit(current_module);
             main_module:=current_module;
             current_module.state:=ms_compile;
           end;
         if not(assigned(current_module) and
                (current_module.state in [ms_compile,ms_second_compile])) then
           internalerror(200212281);

         { Load current state from the init values }
         current_settings:=init_settings;

         { load current asmdata from current_module }
         current_asmdata:=TAsmData(current_module.asmdata);

         { startup scanner and load the first file }
         current_scanner:=tscannerfile.Create(filename);
         current_scanner.firstfile;
         current_module.scanner:=current_scanner;

         { init macros before anything in the file is parsed.}
         current_module.localmacrosymtable:= tmacrosymtable.create(false);
         macrosymtablestack.push(initialmacrosymtable);
         macrosymtablestack.push(current_module.localmacrosymtable);

         { read the first token }
         current_scanner.readtoken(false);

         { this is set to false if a unit needs to wait for other units }
         finished:=true;

         { If the compile level > 1 we get a nice "unit expected" error
           message if we are trying to use a program as unit.}
         try
           try
             if (token=_UNIT) or (compile_level>1) then
               begin
                 current_module.is_unit:=true;
                 finished:=proc_unit;
               end
             else if (token=_ID) and (idtoken=_PACKAGE) then
               begin
                 current_module.IsPackage:=true;
                 proc_package;
               end
             else
               proc_program(token=_LIBRARY);
           except
             on ECompilerAbort do
               raise;
             on Exception do
               begin
                 { Generate exception_raised message,
                   but avoid multiple messages by
                   guarding with exception_raised global variable }
                 if not exception_raised then
                   begin
                     exception_raised:=true;
                     Message(general_e_exception_raised);
                   end;
                 raise;
               end;
           end;

           { the program or the unit at the command line should not need to wait
             for other units }
           if (compile_level=1) and not finished then
             internalerror(2012091901);
         finally
           if assigned(current_module) then
             begin
               if finished then
                 current_module.end_of_parsing
               else
                 begin
                   { these are saved in the unit's state and thus can be set to
                     Nil again as would be done by tmodule.end_of_parsing }
                   macrosymtablestack:=nil;
                   symtablestack:=nil;
                   if current_scanner=current_module.scanner then
                     current_scanner:=nil;
                 end;
             end;

            if (compile_level=1) and
               (status.errorcount=0) then
              { Write Browser Collections }
              do_extractsymbolinfo;

<<<<<<< HEAD
            restore_global_state(olddata^,false);

            { Restore all locally modified warning messages }
            RestoreLocalVerbosity(current_settings.pmessage);
            current_exceptblock:=0;
            exceptblockcounter:=0;

=======
            with olddata^ do
              begin
                { restore scanner }
                c:=oldc;
                pattern:=oldpattern;
                orgpattern:=oldorgpattern;
                token:=oldtoken;
                idtoken:=oldidtoken;
                current_tokenpos:=oldtokenpos;
                block_type:=old_block_type;
                { restore cg }
                parse_only:=oldparse_only;
                { restore symtable state }
                symtablestack:=oldsymtablestack;
                macrosymtablestack:=oldmacrosymtablestack;
                current_procinfo:=oldcurrent_procinfo;
                current_filepos:=oldcurrent_filepos;
                current_settings:=old_settings;
                aktexceptblock:=0;
                exceptblockcounter:=0;
              end;
>>>>>>> graemeg/fixes_2_2
            { Shut down things when the last file is compiled succesfull }
            if (compile_level=1) and
                (status.errorcount=0) then
              begin
                parser_current_file:='';
                { Close script }
                if (not AsmRes.Empty) then
                begin
                  Message1(exec_i_closing_script,AsmRes.Fn);
                  AsmRes.WriteToDisk;
                end;
              end;

          { free now what we did not free earlier in
            proc_program PM }
          if (compile_level=1) and needsymbolinfo then
            begin
              hp:=tmodule(loaded_units.first);
              while assigned(hp) do
               begin
                 hp2:=tmodule(hp.next);
                 if (hp<>current_module) then
                   begin
                     loaded_units.remove(hp);
                     hp.free;
                   end;
                 hp:=hp2;
               end;
              { free also unneeded units we didn't free before }
              unloaded_units.Clear;
             end;
           dec(compile_level);
<<<<<<< HEAD
<<<<<<< HEAD
           { If used units are compiled current_module is already the same as
             the stored module. Now if the unit is not finished its scanner is
             not yet freed and thus set_current_module would reopen the scanned
             file which will result in pointing to the wrong position in the
             file. In the normal case current_scanner and current_module.scanner
             would be Nil, thus nothing bad would happen }
           if olddata^.old_current_module<>current_module then
             set_current_module(olddata^.old_current_module);

           FreeLocalVerbosity(current_settings.pmessage);
=======
           set_current_module(olddata^.old_current_module);
>>>>>>> graemeg/fixes_2_2
=======
           set_current_module(olddata^.old_current_module);
>>>>>>> origin/fixes_2_2

           dispose(olddata);
         end;
    end;

end.
