{
    Copyright (c) 2008 by Jonas Maebe

    Whole program optimisation information collection base class

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

unit wpobase;

{$i fpcdefs.inc}

interface

uses
  globtype,
  cclasses,
  symtype;

type
  { the types of available whole program optimization }
  twpotype = (wpo_devirtualization_context_insensitive,wpo_live_symbol_information);
const
  wpo2str: array[twpotype] of string[16] = ('devirtualization','symbol liveness');

type
  { ************************************************************************* }
  { ******************** General base classes/interfaces ******************** }
  { ************************************************************************* }

  { interface to reading a section from a file with wpo info }
  twposectionreaderintf = interface
    ['{51BE3F89-C9C5-4965-9C83-AE7490C92E3E}']
    function sectiongetnextline(out s: string): boolean;
  end;


  { interface to writing sections to a file with wpoinfo }
  twposectionwriterintf = interface
    ['{C056F0DD-62B1-4612-86C7-2D39944C4437}']
    procedure startsection(const name: string);
    procedure sectionputline(const s: string);
  end;


  { base class for wpo information stores }

  { twpocomponentbase }

  twpocomponentbase = class
   public
    constructor create; reintroduce; virtual;

    { type of whole program optimization information collected/provided by
      this class
    }
    class function getwpotype: twpotype; virtual; abstract;

    { whole program optimizations for which this class generates information }
    class function generatesinfoforwposwitches: twpoptimizerswitches; virtual; abstract;

    { whole program optimizations performed by this class }
    class function performswpoforswitches: twpoptimizerswitches; virtual; abstract;

    { returns the name of the section parsed by this class }
    class function sectionname: shortstring; virtual; abstract;

    { checks whether the compiler options are compatible with this
      optimization (default: don't check anything)
    }
    class procedure checkoptions; virtual;

    { loads the information pertinent to this whole program optimization from
      the current section being processed by reader
    }
    procedure loadfromwpofilesection(reader: twposectionreaderintf); virtual; abstract;

    { stores the information of this component to a file in a format that can
      be loaded again using loadfromwpofilesection()
    }
    procedure storewpofilesection(writer: twposectionwriterintf); virtual; abstract;

    { extracts the information pertinent to this whole program optimization
      from the current compiler state (loaded units, ...)
    }
    procedure constructfromcompilerstate; virtual; abstract;
  end;

  twpocomponentbaseclass = class of twpocomponentbase;


  { forward declaration of overall wpo info manager class }

  twpoinfomanagerbase = class;

  { ************************************************************************* }
  { ** Information created per unit for use during subsequent compilation *** }
  { ************************************************************************* }

  { information about called vmt entries for a class }
  tcalledvmtentries = class
   protected
    { the class }
    fobjdef: tdef;
    fobjdefderef: tderef;
    { the vmt entries }
    fcalledentries: tbitset;
   public
    constructor create(_objdef: tdef; nentries: longint);
    constructor ppuload(ppufile: tcompilerppufile);
    destructor destroy; override;
    procedure ppuwrite(ppufile: tcompilerppufile);

    procedure buildderef;
    procedure buildderefimpl;
    procedure deref;
    procedure derefimpl;

    property objdef: tdef read fobjdef write fobjdef;
    property objdefderef: tderef read fobjdefderef write fobjdefderef;
    property calledentries: tbitset read fcalledentries write fcalledentries;
  end;


  { base class of information collected per unit. Still needs to be
    generalised for different kinds of wpo information, currently specific
    to devirtualization.
  }

  tunitwpoinfobase = class
   protected
    { created object types }
    fcreatedobjtypes: tfpobjectlist;
    { objectdefs pointed to by created classrefdefs }
    fcreatedclassrefobjtypes: tfpobjectlist;
    { objtypes potentially instantiated by fcreatedclassrefobjtypes
      (objdectdefs pointed to by classrefdefs that are
       passed as a regular parameter, loaded in a variable, ...
       so they can end up in a classrefdef var and be instantiated)
    }
    fmaybecreatedbyclassrefdeftypes: tfpobjectlist;

    { called virtual methods for all classes (hashed by mangled classname,
      entries bitmaps indicating which vmt entries per class are called --
      tcalledvmtentries)
    }
    fcalledvmtentries: tfphashlist;
   public
    constructor create; reintroduce; virtual;
    destructor destroy; override;

    property createdobjtypes: tfpobjectlist read fcreatedobjtypes;
    property createdclassrefobjtypes: tfpobjectlist read fcreatedclassrefobjtypes;
    property maybecreatedbyclassrefdeftypes: tfpobjectlist read fmaybecreatedbyclassrefdeftypes;
    property calledvmtentries: tfphashlist read fcalledvmtentries;

    procedure addcreatedobjtype(def: tdef);
    procedure addcreatedobjtypeforclassref(def: tdef);
    procedure addmaybecreatedbyclassref(def: tdef);
    procedure addcalledvmtentry(def: tdef; index: longint);

    { resets the "I've been registered with wpo" flags for all defs in the
      above lists }
    procedure resetdefs;
  end;

  { ************************************************************************* }
  { **** Total information created for use during subsequent compilation **** }
  { ************************************************************************* }

  { class to create a file with wpo information }

  { tavailablewpofilewriter }

  twpofilewriter = class(tobject,twposectionwriterintf)
   private
    { array of class *instances* that wish to be written out to the
      whole program optimization feedback file
    }
    fsectioncontents: tfpobjectlist;

    ffilename: tcmdstr;
    foutputfile: text;

   public
    constructor create(const fn: tcmdstr);
    destructor destroy; override;

    procedure writefile;

    { starts a new section with name "name" }
    procedure startsection(const name: string);
    { writes s to the wpo file }
    procedure sectionputline(const s: string);

    { register a component instance that needs to be written
      to the wpo feedback file
    }
    procedure registerwpocomponent(component: twpocomponentbase);
  end;

  { ************************************************************************* }
  { ************ Information for use during current compilation ************* }
  { ************************************************************************* }

  { class to read a file with wpo information }
  twpofilereader = class(tobject,twposectionreaderintf)
   private
    ffilename: tcmdstr;
    flinenr: longint;
    finputfile: text;
    fcurline: string;
    fusecurline: boolean;

    { destination for the read information }
    fdest: twpoinfomanagerbase;

    function getnextnoncommentline(out s: string): boolean;
   public

     constructor create(const fn: tcmdstr; dest: twpoinfomanagerbase);
     destructor destroy; override;

     { processes the wpo info in the file }
     procedure processfile;

     { returns next line of the current section in s, and false if no more
       lines in the current section
     }
     function sectiongetnextline(out s: string): boolean;
  end;


  { ************************************************************************* }
  { ******* Specific kinds of whole program optimization components ********* }
  { ************************************************************************* }

  { method devirtualisation }
  twpodevirtualisationhandler = class(twpocomponentbase)
    { checks whether procdef (a procdef for a virtual method) can be replaced with
      a static call when it's called as objdef.procdef, and if so returns the
      mangled name in staticname.
    }
    function staticnameforcallingvirtualmethod(objdef, procdef: tdef; out staticname: TSymStr): boolean; virtual; abstract;
    { checks whether procdef (a procdef for a virtual method) can be replaced with
      a different procname in the vmt of objdef, and if so returns the new
      mangledname in staticname
    }
    function staticnameforvmtentry(objdef, procdef: tdef; out staticname: TSymStr): boolean; virtual; abstract;
  end;

  twpodeadcodehandler = class(twpocomponentbase)
    { checks whether a mangledname was removed as dead code from the final
      binary (WARNING: must *not* be called for functions marked as inline,
      since if all call sites are inlined, it won't appear in the final
      binary but nevertheless is still necessary!)
    }
    function symbolinfinalbinary(const s: shortstring): boolean; virtual; abstract;
  end;


  { ************************************************************************* }
  { ************ Collection of all instances of wpo components ************** }
  { ************************************************************************* }

  { class doing all the bookkeeping for everything  }

  twpoinfomanagerbase = class
   private
    { array of classrefs of handler classes for the various kinds of whole
      program optimizations that we support
    }
    fwpocomponents: tfphashlist;

    freader: twpofilereader;
    fwriter: twpofilewriter;
   public
    { instances of the various optimizers/information collectors (for
      information used during this compilation)
    }
    wpoinfouse: array[twpotype] of twpocomponentbase;

    { register a whole program optimization class type }
    procedure registerwpocomponentclass(wpocomponent: twpocomponentbaseclass);

    { get the program optimization class type that can parse the contents
      of the section with name "secname" in the wpo feedback file
    }
    function gethandlerforsection(const secname: string): twpocomponentbaseclass;

    { tell all instantiated wpo component classes to collect the information
      from the global compiler state that they need (done at the very end of
      the compilation process)
    }
    procedure extractwpoinfofromprogram;

    { set the name of the feedback file from which all whole-program information
      to be used during the current compilation will be read
    }
    procedure setwpoinputfile(const fn: tcmdstr);

    { set the name of the feedback file to which all whole-program information
      collected during the current compilation will be written
    }
    procedure setwpooutputfile(const fn: tcmdstr);

    { check whether the specified wpo options (-FW/-Fw/-OW/-Ow) are complete
      and sensical, and parse the wpo feedback file specified with
      setwpoinputfile
    }
    procedure parseandcheckwpoinfo;

    { routines accessing the optimizer information }
    { 1) devirtualization at the symbol name level }
    function can_be_devirtualized(objdef, procdef: tdef; out name: TSymStr): boolean; virtual; abstract;
    { 2) optimal replacement method name in vmt }
    function optimized_name_for_vmt(objdef, procdef: tdef; out name: TSymStr): boolean; virtual; abstract;
    { 3) does a symbol appear in the final binary (i.e., not removed by dead code stripping/smart linking).
        WARNING: do *not* call for inline functions/procedures/methods/...
    }
    function symbol_live(const name: shortstring): boolean; virtual; abstract;

    constructor create; reintroduce;
    destructor destroy; override;
  end;


  var
    wpoinfomanager: twpoinfomanagerbase;

implementation

  uses
    globals,
    cutils,
    sysutils,
    symdef,
    verbose;


  { tcreatedwpoinfobase }

  constructor tunitwpoinfobase.create;
    begin
      fcreatedobjtypes:=tfpobjectlist.create(false);
      fcreatedclassrefobjtypes:=tfpobjectlist.create(false);
      fmaybecreatedbyclassrefdeftypes:=tfpobjectlist.create(false);
      fcalledvmtentries:=tfphashlist.create;
    end;


  destructor tunitwpoinfobase.destroy;
    var
      i: longint;
    begin
      { don't call resetdefs here, because the defs may have been freed
        already }
      fcreatedobjtypes.free;
      fcreatedobjtypes:=nil;
      fcreatedclassrefobjtypes.free;
      fcreatedclassrefobjtypes:=nil;
      fmaybecreatedbyclassrefdeftypes.free;
      fmaybecreatedbyclassrefdeftypes:=nil;

      { may not be assigned in case the info was loaded from a ppu and we
        are not generating a wpo feedback file (see tunitwpoinfo.ppuload)
      }
      if assigned(fcalledvmtentries) then
        begin
          for i:=0 to fcalledvmtentries.count-1 do
            tcalledvmtentries(fcalledvmtentries[i]).free;
          fcalledvmtentries.free;
          fcalledvmtentries:=nil;
        end;

      inherited destroy;
    end;
<<<<<<< HEAD


=======
    
    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  procedure tunitwpoinfobase.resetdefs;
    var
      i: ptrint;
    begin
      if assigned(fcreatedobjtypes) then
        for i:=0 to fcreatedobjtypes.count-1 do
          tobjectdef(fcreatedobjtypes[i]).created_in_current_module:=false;
      if assigned(fcreatedclassrefobjtypes) then
        for i:=0 to fcreatedclassrefobjtypes.count-1 do
          tobjectdef(fcreatedclassrefobjtypes[i]).classref_created_in_current_module:=false;
      if assigned(fmaybecreatedbyclassrefdeftypes) then
        for i:=0 to fmaybecreatedbyclassrefdeftypes.count-1 do
          tobjectdef(fmaybecreatedbyclassrefdeftypes[i]).maybe_created_in_current_module:=false;
    end;


  procedure tunitwpoinfobase.addcreatedobjtype(def: tdef);
    begin
      fcreatedobjtypes.add(def);
    end;


  procedure tunitwpoinfobase.addcreatedobjtypeforclassref(def: tdef);
    begin
      fcreatedclassrefobjtypes.add(def);
    end;


  procedure tunitwpoinfobase.addmaybecreatedbyclassref(def: tdef);
    begin
      fmaybecreatedbyclassrefdeftypes.add(def);
    end;


  procedure tunitwpoinfobase.addcalledvmtentry(def: tdef; index: longint);
    var
      entries: tcalledvmtentries;
      key: shortstring;
    begin
      key:=tobjectdef(def).vmt_mangledname;
      entries:=tcalledvmtentries(fcalledvmtentries.find(key));
      if not assigned(entries) then
        begin
          entries:=tcalledvmtentries.create(def,tobjectdef(def).vmtentries.count);
          fcalledvmtentries.add(key,entries);
        end;
      entries.calledentries.include(index);
    end;


  { twpofilereader }

  function twpofilereader.getnextnoncommentline(out s: string):
    boolean;
    begin
      if (fusecurline) then
        begin
          s:=fcurline;
          fusecurline:=false;
          result:=true;
          exit;
        end;
      repeat
        readln(finputfile,s);
        if (s='') and
           eof(finputfile) then
          begin
            result:=false;
            exit;
          end;
        inc(flinenr);
      until (s='') or
            (s[1]<>'#');
      result:=true;
    end;

  constructor twpofilereader.create(const fn: tcmdstr; dest: twpoinfomanagerbase);
    begin
      if not FileExists(fn) or
         { FileExists also returns true for directories }
         DirectoryExists(fn) then
        begin
          cgmessage1(wpo_cant_find_file,fn);
          exit;
        end;
      assign(finputfile,fn);
      ffilename:=fn;

      fdest:=dest;
    end;

  destructor twpofilereader.destroy;
    begin
      inherited destroy;
    end;

  procedure twpofilereader.processfile;
    var
      sectionhandler: twpocomponentbaseclass;
      i: longint;
      wpotype: twpotype;
      s,
      sectionname: string;
    begin
      cgmessage1(wpo_begin_processing,ffilename);
      reset(finputfile);
      flinenr:=0;
      while getnextnoncommentline(s) do
        begin
          if (s='') then
            continue;
          { format: "% sectionname" }
          if (s[1]<>'%') then
            begin
              cgmessage2(wpo_expected_section,tostr(flinenr),s);
              break;
            end;
          for i:=2 to length(s) do
            if (s[i]<>' ') then
              break;
          sectionname:=copy(s,i,255);

          { find handler for section and process }
          sectionhandler:=fdest.gethandlerforsection(sectionname);
          if assigned(sectionhandler) then
            begin
              wpotype:=sectionhandler.getwpotype;
              cgmessage2(wpo_found_section,sectionname,wpo2str[wpotype]);
              { do we need this information? }
              if ((sectionhandler.performswpoforswitches * init_settings.dowpoptimizerswitches) <> []) then
                begin
                  { did some other section already generate this type of information? }
                  if assigned(fdest.wpoinfouse[wpotype]) then
                    begin
                      cgmessage2(wpo_duplicate_wpotype,wpo2str[wpotype],sectionname);
                      fdest.wpoinfouse[wpotype].free;
                    end;
                  { process the section }
                  fdest.wpoinfouse[wpotype]:=sectionhandler.create;
                  twpocomponentbase(fdest.wpoinfouse[wpotype]).loadfromwpofilesection(self);
                end
              else
                begin
                  cgmessage1(wpo_skipping_unnecessary_section,sectionname);
                  { skip the current section }
                  while sectiongetnextline(s) do
                    ;
                end;
            end
          else
            begin
              cgmessage1(wpo_no_section_handler,sectionname);
              { skip the current section }
              while sectiongetnextline(s) do
                ;
            end;
        end;
      close(finputfile);
      cgmessage1(wpo_end_processing,ffilename);
    end;

  function twpofilereader.sectiongetnextline(out s: string): boolean;
    begin
      result:=getnextnoncommentline(s);
      if not result then
        exit;
      { start of new section? }
      if (s<>'') and
         (s[1]='%') then
        begin
          { keep read line for next call to getnextnoncommentline() }
          fcurline:=s;
          fusecurline:=true;
          result:=false;
        end;
    end;


  { twpocomponentbase }

  constructor twpocomponentbase.create;
    begin
      { do nothing }
    end;


  class procedure twpocomponentbase.checkoptions;
    begin
      { do nothing }
    end;

  { twpofilewriter }

  constructor twpofilewriter.create(const fn: tcmdstr);
    begin
      assign(foutputfile,fn);
      ffilename:=fn;
      fsectioncontents:=tfpobjectlist.create(true);
    end;

  destructor twpofilewriter.destroy;
    begin
      fsectioncontents.free;
      inherited destroy;
    end;

  procedure twpofilewriter.writefile;
    var
      i: longint;
    begin
      {$push}{$i-}
      rewrite(foutputfile);
      {$pop}
      if (ioresult <> 0) then
        begin
          cgmessage1(wpo_cant_create_feedback_file,ffilename);
          exit;
        end;
      for i:=0 to fsectioncontents.count-1 do
        twpocomponentbase(fsectioncontents[i]).storewpofilesection(self);
      close(foutputfile);
    end;

  procedure twpofilewriter.startsection(const name: string);
    begin
      writeln(foutputfile,'% ',name);
    end;

  procedure twpofilewriter.sectionputline(const s: string);
    begin
      writeln(foutputfile,s);
    end;

  procedure twpofilewriter.registerwpocomponent(
    component: twpocomponentbase);
    begin
      fsectioncontents.add(component);
    end;

{ twpoinfomanagerbase }

  procedure twpoinfomanagerbase.registerwpocomponentclass(wpocomponent: twpocomponentbaseclass);
    begin
      fwpocomponents.add(wpocomponent.sectionname,wpocomponent);
    end;


  function twpoinfomanagerbase.gethandlerforsection(const secname: string
      ): twpocomponentbaseclass;
    begin
      result:=twpocomponentbaseclass(fwpocomponents.find(secname));
    end;

  procedure twpoinfomanagerbase.setwpoinputfile(const fn: tcmdstr);
    begin
      freader:=twpofilereader.create(fn,self);
    end;

  procedure twpoinfomanagerbase.setwpooutputfile(const fn: tcmdstr);
    begin
      fwriter:=twpofilewriter.create(fn);
    end;

  procedure twpoinfomanagerbase.parseandcheckwpoinfo;
    var
      i: longint;
    begin
      { error if we don't have to optimize yet have an input feedback file }
      if (init_settings.dowpoptimizerswitches=[]) and
         assigned(freader) then
        begin
          cgmessage(wpo_input_without_info_use);
          exit;
        end;

      { error if we have to optimize yet don't have an input feedback file }
      if (init_settings.dowpoptimizerswitches<>[]) and
         not assigned(freader) then
        begin
          cgmessage(wpo_no_input_specified);
          exit;
        end;

      { if we have to generate wpo information, check that a file has been
        specified and that we have something to write to it
      }
      if (init_settings.genwpoptimizerswitches<>[]) and
         not assigned(fwriter) then
        begin
          cgmessage(wpo_no_output_specified);
          exit;
        end;

      if (init_settings.genwpoptimizerswitches=[]) and
         assigned(fwriter) then
        begin
          cgmessage(wpo_output_without_info_gen);
          exit;
        end;

      { now read the input feedback file }
      if assigned(freader) then
        begin
          freader.processfile;
          freader.free;
          freader:=nil;
        end;

      { and for each specified optimization check whether the input feedback
        file contained the necessary information
      }
      if (([cs_wpo_devirtualize_calls,cs_wpo_optimize_vmts] * init_settings.dowpoptimizerswitches) <> []) and
         not assigned(wpoinfouse[wpo_devirtualization_context_insensitive]) then
        begin
          cgmessage1(wpo_not_enough_info,wpo2str[wpo_devirtualization_context_insensitive]);
          exit;
        end;

      if (cs_wpo_symbol_liveness in init_settings.dowpoptimizerswitches) and
         not assigned(wpoinfouse[wpo_live_symbol_information]) then
        begin
          cgmessage1(wpo_not_enough_info,wpo2str[wpo_live_symbol_information]);
          exit;
        end;

      { perform pre-checking to ensure there are no known incompatibilities between
        the selected optimizations and other switches
      }
      for i:=0 to fwpocomponents.count-1 do
        if (twpocomponentbaseclass(fwpocomponents[i]).generatesinfoforwposwitches*init_settings.genwpoptimizerswitches)<>[] then
          twpocomponentbaseclass(fwpocomponents[i]).checkoptions
    end;

  procedure twpoinfomanagerbase.extractwpoinfofromprogram;
    var
      i: longint;
      info: twpocomponentbase;
    begin
      { if don't have to write anything, fwriter has not been created }
      if not assigned(fwriter) then
        exit;

      { let all wpo components gather the necessary info from the compiler state }
      for i:=0 to fwpocomponents.count-1 do
        if (twpocomponentbaseclass(fwpocomponents[i]).generatesinfoforwposwitches*current_settings.genwpoptimizerswitches)<>[] then
          begin
            info:=twpocomponentbaseclass(fwpocomponents[i]).create;
            info.constructfromcompilerstate;
            fwriter.registerwpocomponent(info);
          end;
      { and write their info to disk }
      fwriter.writefile;
      fwriter.free;
      fwriter:=nil;
    end;

  constructor twpoinfomanagerbase.create;
    begin
      inherited create;
      fwpocomponents:=tfphashlist.create;
    end;

  destructor twpoinfomanagerbase.destroy;
    var
      i: twpotype;
    begin
      freader.free;
      freader:=nil;
      fwriter.free;
      fwriter:=nil;
      fwpocomponents.free;
      fwpocomponents:=nil;
      for i:=low(wpoinfouse) to high(wpoinfouse) do
        if assigned(wpoinfouse[i]) then
          wpoinfouse[i].free;
      inherited destroy;
    end;

  { tcalledvmtentries }

  constructor tcalledvmtentries.create(_objdef: tdef; nentries: longint);
    begin
      objdef:=_objdef;
      calledentries:=tbitset.create(nentries);
    end;


  constructor tcalledvmtentries.ppuload(ppufile: tcompilerppufile);
    var
      len: longint;
    begin
      ppufile.getderef(fobjdefderef);
      len:=ppufile.getlongint;
      calledentries:=tbitset.create_bytesize(len);
      if (len <> calledentries.datasize) then
        internalerror(2009060301);
      ppufile.readdata(calledentries.data^,len);
    end;


  destructor tcalledvmtentries.destroy;
    begin
      fcalledentries.free;
      inherited destroy;
    end;


  procedure tcalledvmtentries.ppuwrite(ppufile: tcompilerppufile);
    begin
      ppufile.putderef(objdefderef);
      ppufile.putlongint(calledentries.datasize);
      ppufile.putdata(calledentries.data^,calledentries.datasize);
    end;


  procedure tcalledvmtentries.buildderef;
    begin
      objdefderef.build(objdef);
    end;


  procedure tcalledvmtentries.buildderefimpl;
    begin
    end;


  procedure tcalledvmtentries.deref;
    begin
      objdef:=tdef(objdefderef.resolve);
    end;


  procedure tcalledvmtentries.derefimpl;
    begin
    end;

end.
