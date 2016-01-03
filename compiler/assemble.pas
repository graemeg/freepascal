{
    Copyright (c) 1998-2004 by Peter Vreman

    This unit handles the assemblerfile write and assembler calls of FPC

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
{# @abstract(This unit handles the assembler file write and assembler calls of FPC)
   Handles the calls to the actual external assemblers, as well as the generation
   of object files for smart linking. Also contains the base class for writing
   the assembler statements to file.
}
unit assemble;

{$i fpcdefs.inc}

interface


    uses
      SysUtils,
<<<<<<< HEAD
<<<<<<< HEAD
      systems,globtype,globals,aasmbase,aasmtai,aasmdata,ogbase,owbase,finput;
=======
      systems,globtype,globals,aasmbase,aasmtai,aasmdata,ogbase,finput;
>>>>>>> graemeg/fixes_2_2
=======
      systems,globtype,globals,aasmbase,aasmtai,aasmdata,ogbase,finput;
>>>>>>> origin/fixes_2_2

    const
       { maximum of aasmoutput lists there will be }
       maxoutputlists = ord(high(tasmlisttype))+1;
       { buffer size for writing the .s file }
       AsmOutSize=32768*4;

    type
      TAssembler=class(TObject)
      public
      {assembler info}
        asminfo     : pasminfo;
      {filenames}
        path        : TPathStr;
        name        : string;
        AsmFileName,         { current .s and .o file }
        ObjFileName,
        ppufilename  : TPathStr;
        asmprefix    : string;
        SmartAsm     : boolean;
        SmartFilesCount,
        SmartHeaderCount : longint;
        Constructor Create(info: pasminfo; smart:boolean);virtual;
        Destructor Destroy;override;
        procedure NextSmartName(place:tcutplace);
        procedure MakeObject;virtual;abstract;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      TExternalAssembler = class;

      IExternalAssemblerOutputFileDecorator=interface
        function LinePrefix: AnsiString;
        function LinePostfix: AnsiString;
        function LineFilter(const s: AnsiString): AnsiString;
      end;

      TExternalAssemblerOutputFile=class
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      {# This is the base class which should be overridden for each each
         assembler writer. It is used to actually assembler a file,
         and write the output to the assembler file.
      }
      TExternalAssembler=class(TAssembler)
>>>>>>> graemeg/cpstrnew
      private
        fdecorator: IExternalAssemblerOutputFileDecorator;
      protected
        owner: TExternalAssembler;
      {outfile}
        AsmSize,
        AsmStartSize,
        outcnt   : longint;
        outbuf   : array[0..AsmOutSize-1] of char;
        outfile  : file;
<<<<<<< HEAD
        fioerror : boolean;
        linestart: boolean;

        Procedure AsmClear;
        Procedure MaybeAddLinePrefix;
        Procedure MaybeAddLinePostfix;

        Procedure AsmWriteAnsiStringUnfiltered(const s: ansistring);
      public
        Constructor Create(_owner: TExternalAssembler);

        Procedure RemoveAsm;virtual;
        Procedure AsmFlush;

        { mark the current output as the "empty" state (i.e., it only contains
          headers/directives etc }
        Procedure MarkEmpty;
        { clears the assembler output if nothing was added since it was marked
          as empty, and returns whether it was empty }
        function ClearIfEmpty: boolean;
        { these routines will write the filtered version of their argument
          according to the current decorator }
        procedure AsmWriteFiltered(const c:char);
        procedure AsmWriteFiltered(const s:string);
        procedure AsmWriteFiltered(const s:ansistring);
        procedure AsmWriteFiltered(p:pchar; len: longint);

        {# Write a string to the assembler file }
        Procedure AsmWrite(const c:char);
        Procedure AsmWrite(const s:string);
        Procedure AsmWrite(const s:ansistring);

        {# Write a string to the assembler file }
        Procedure AsmWritePChar(p:pchar);

        {# Write a string to the assembler file followed by a new line }
        Procedure AsmWriteLn(const c:char);
        Procedure AsmWriteLn(const s:string);
        Procedure AsmWriteLn(const s:ansistring);

        {# Write a new line to the assembler file }
        Procedure AsmLn; virtual;

        procedure AsmCreate(Aplace:tcutplace);
        procedure AsmClose;

        property ioerror: boolean read fioerror;
        property decorator: IExternalAssemblerOutputFileDecorator read fdecorator write fdecorator;
      end;

      {# This is the base class which should be overridden for each each
         assembler writer. It is used to actually assembler a file,
         and write the output to the assembler file.
      }
      TExternalAssembler=class(TAssembler)
      private
       { output writer }
        fwriter: TExternalAssemblerOutputFile;
        ffreewriter: boolean;

        procedure CreateSmartLinkPath(const s:TPathStr);
      protected
=======
        ioerror : boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      {input source info}
        lastfileinfo : tfileposinfo;
        infile,
        lastinfile   : tinputfile;
      {last section type written}
        lastsectype : TAsmSectionType;
<<<<<<< HEAD
<<<<<<< HEAD
        procedure WriteSourceLine(hp: tailineinfo);
        procedure WriteTempalloc(hp: tai_tempalloc);
        procedure WriteRealConstAsBytes(hp: tai_realconst; const dbdir: string; do_line: boolean);
        function single2str(d : single) : string; virtual;
        function double2str(d : double) : string; virtual;
        function extended2str(e : extended) : string; virtual;
        Function DoPipe:boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      public

        {# Returns the complete path and executable name of the assembler
           program.

           It first tries looking in the UTIL directory if specified,
           otherwise it searches in the free pascal binary directory, in
           the current working directory and then in the  directories
           in the $PATH environment.}
        Function  FindAssembler:string;

        {# Actually does the call to the assembler file. Returns false
           if the assembling of the file failed.}
        Function  CallAssembler(const command:string; const para:TCmdStr):Boolean;

        Function  DoAssemble:boolean;virtual;

<<<<<<< HEAD
=======
        {# Write a new line to the assembler file }
        Procedure AsmLn;

        procedure AsmCreate(Aplace:tcutplace);
        procedure AsmClose;

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
        {# This routine should be overridden for each assembler, it is used
           to actually write the abstract assembler stream to file.}
        procedure WriteTree(p:TAsmList);virtual;

        {# This routine should be overridden for each assembler, it is used
           to actually write all the different abstract assembler streams
           by calling for each stream type, the @var(WriteTree) method.}
        procedure WriteAsmList;virtual;

        {# Constructs the command line for calling the assembler }
        function MakeCmdLine: TCmdStr; virtual;
      public
        Constructor Create(info: pasminfo; smart: boolean);override;
        Constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
        procedure MakeObject;override;
        destructor Destroy; override;

        property writer: TExternalAssemblerOutputFile read fwriter;
      end;
      TExternalAssemblerClass = class of TExternalAssembler;

      { TInternalAssembler }

      { TInternalAssembler }

      { TInternalAssembler }

      { TInternalAssembler }

      { TInternalAssembler }

      TInternalAssembler=class(TAssembler)
      private
        FCObjOutput : TObjOutputclass;
        FCInternalAr : TObjectWriterClass;
        { the aasmoutput lists that need to be processed }
        lists        : byte;
        list         : array[1..maxoutputlists] of TAsmList;
        { current processing }
        currlistidx  : byte;
        currlist     : TAsmList;
        procedure WriteStab(p:pchar);
        function  MaybeNextList(var hp:Tai):boolean;
        function  SetIndirectToSymbol(hp: Tai; const indirectname: string): Boolean;
        function  TreePass0(hp:Tai):Tai;
        function  TreePass1(hp:Tai):Tai;
        function  TreePass2(hp:Tai):Tai;
        procedure writetree;
        procedure writetreesmart;
      protected
        ObjData   : TObjData;
        ObjOutput : tObjOutput;
        property CObjOutput:TObjOutputclass read FCObjOutput write FCObjOutput;
        property CInternalAr : TObjectWriterClass read FCInternalAr write FCInternalAr;
      public
        constructor Create(info: pasminfo; smart: boolean);override;
        destructor  destroy;override;
        procedure MakeObject;override;
      end;

    TAssemblerClass = class of TAssembler;

    Procedure GenerateAsm(smart:boolean);

    { get an instance of an external GNU-style assembler that is compatible
      with the current target, reusing an existing writer. Used by the LLVM
      target to write inline assembler }
    function GetExternalGnuAssemblerWithAsmInfoWriter(info: pasminfo; wr: TExternalAssemblerOutputFile): TExternalAssembler;

    procedure RegisterAssembler(const r:tasminfo;c:TAssemblerClass);


Implementation

    uses
{$ifdef hasunix}
      unix,
{$endif}
      cutils,cfileutl,
{$ifdef memdebug}
      cclasses,
{$endif memdebug}
      script,fmodule,verbose,
{$if defined(m68k) or defined(arm)}
      cpuinfo,
{$endif m68k or arm}
      aasmcpu,
      owar,owomflib
      ;

    var
      CAssembler : array[tasm] of TAssemblerClass;

    function fixline(s:string):string;
     {
       return s with all leading and ending spaces and tabs removed
     }
      var
        i,j,k : integer;
      begin
        i:=length(s);
        while (i>0) and (s[i] in [#9,' ']) do
          dec(i);
        j:=1;
        while (j<i) and (s[j] in [#9,' ']) do
          inc(j);
        for k:=j to i do
          if s[k] in [#0..#31,#127..#255] then
            s[k]:='.';
        fixline:=Copy(s,j,i-j+1);
      end;

{*****************************************************************************
                                   TAssembler
*****************************************************************************}

    Constructor TAssembler.Create(info: pasminfo; smart: boolean);
      begin
        asminfo:=info;
      { load start values }
        AsmFileName:=current_module.AsmFilename;
        ObjFileName:=current_module.ObjFileName;
        name:=Lower(current_module.modulename^);
        path:=current_module.outputpath;
        asmprefix := current_module.asmprefix^;
        if current_module.outputpath = '' then
          ppufilename := ''
        else
          ppufilename := current_module.ppufilename;
        SmartAsm:=smart;
        SmartFilesCount:=0;
        SmartHeaderCount:=0;
        SmartLinkOFiles.Clear;
      end;


    Destructor TAssembler.Destroy;
      begin
      end;


    procedure TAssembler.NextSmartName(place:tcutplace);
      var
        s : string;
      begin
        inc(SmartFilesCount);
        if SmartFilesCount>999999 then
         Message(asmw_f_too_many_asm_files);
        case place of
          cut_begin :
            begin
              inc(SmartHeaderCount);
              s:=asmprefix+tostr(SmartHeaderCount)+'h';
            end;
          cut_normal :
            s:=asmprefix+tostr(SmartHeaderCount)+'s';
          cut_end :
            s:=asmprefix+tostr(SmartHeaderCount)+'t';
        end;
        AsmFileName:=Path+FixFileName(s+tostr(SmartFilesCount)+target_info.asmext);
        ObjFileName:=Path+FixFileName(s+tostr(SmartFilesCount)+target_info.objext);
        { insert in container so it can be cleared after the linking }
        SmartLinkOFiles.Insert(ObjFileName);
      end;




{*****************************************************************************
                                 TAssemblerOutputFile
*****************************************************************************}

    procedure TExternalAssemblerOutputFile.RemoveAsm;
      var
        g : file;
      begin
        if cs_asm_leave in current_settings.globalswitches then
         exit;
        if cs_asm_extern in current_settings.globalswitches then
         AsmRes.AddDeleteCommand(owner.AsmFileName)
        else
         begin
           assign(g,owner.AsmFileName);
           {$push} {$I-}
            erase(g);
           {$pop}
           if ioresult<>0 then;
         end;
      end;


    Procedure TExternalAssemblerOutputFile.AsmFlush;
      begin
        if outcnt>0 then
         begin
           { suppress i/o error }
           {$push} {$I-}
           BlockWrite(outfile,outbuf,outcnt);
           {$pop}
           fioerror:=fioerror or (ioresult<>0);
           outcnt:=0;
         end;
      end;

    procedure TExternalAssemblerOutputFile.MarkEmpty;
      begin
        AsmStartSize:=AsmSize
      end;


    function TExternalAssemblerOutputFile.ClearIfEmpty: boolean;
      begin
        result:=AsmSize=AsmStartSize;
        if result then
         AsmClear;
      end;


    procedure TExternalAssemblerOutputFile.AsmWriteFiltered(const c: char);
      begin
        MaybeAddLinePrefix;
        AsmWriteAnsiStringUnfiltered(decorator.LineFilter(c));
      end;


    procedure TExternalAssemblerOutputFile.AsmWriteFiltered(const s: string);
      begin
        MaybeAddLinePrefix;
        AsmWriteAnsiStringUnfiltered(decorator.LineFilter(s));
      end;


    procedure TExternalAssemblerOutputFile.AsmWriteFiltered(const s: ansistring);
      begin
        MaybeAddLinePrefix;
        AsmWriteAnsiStringUnfiltered(decorator.LineFilter(s));
      end;


    procedure TExternalAssemblerOutputFile.AsmWriteFiltered(p: pchar; len: longint);
      var
        s: ansistring;
      begin
        MaybeAddLinePrefix;
        setlength(s,len);
        move(p^,s[1],len);
        AsmWriteAnsiStringUnfiltered(decorator.LineFilter(s));
      end;


    Procedure TExternalAssemblerOutputFile.AsmClear;
      begin
        outcnt:=0;
      end;


    procedure TExternalAssemblerOutputFile.MaybeAddLinePrefix;
      begin
        if assigned(decorator) and
           linestart then
          begin
            AsmWriteAnsiStringUnfiltered(decorator.LinePrefix);
            linestart:=false;
          end;
      end;


    procedure TExternalAssemblerOutputFile.MaybeAddLinePostfix;
      begin
        if assigned(decorator) and
           not linestart then
          begin
            AsmWriteAnsiStringUnfiltered(decorator.LinePostfix);
            linestart:=true;
          end;
      end;


    procedure TExternalAssemblerOutputFile.AsmWriteAnsiStringUnfiltered(const s: ansistring);
      var
        StartIndex, ToWrite: longint;
      begin
        if s='' then
          exit;
        if OutCnt+length(s)>=AsmOutSize then
         AsmFlush;
        StartIndex:=1;
        ToWrite:=length(s);
        while ToWrite>AsmOutSize do
          begin
            Move(s[StartIndex],OutBuf[OutCnt],AsmOutSize);
            inc(OutCnt,AsmOutSize);
            inc(AsmSize,AsmOutSize);
            AsmFlush;
            inc(StartIndex,AsmOutSize);
            dec(ToWrite,AsmOutSize);
          end;
        Move(s[StartIndex],OutBuf[OutCnt],ToWrite);
        inc(OutCnt,ToWrite);
        inc(AsmSize,ToWrite);
      end;


    constructor TExternalAssemblerOutputFile.Create(_owner: TExternalAssembler);
      begin
        owner:=_owner;
        linestart:=true;
      end;


    Procedure TExternalAssemblerOutputFile.AsmWrite(const c: char);
      begin
        if assigned(decorator) then
          AsmWriteFiltered(c)
        else
          begin
            if OutCnt+1>=AsmOutSize then
             AsmFlush;
            OutBuf[OutCnt]:=c;
            inc(OutCnt);
            inc(AsmSize);
          end;
      end;


    Procedure TExternalAssemblerOutputFile.AsmWrite(const s:string);
      begin
        if s='' then
          exit;
        if assigned(decorator) then
          AsmWriteFiltered(s)
        else
          begin
            if OutCnt+length(s)>=AsmOutSize then
             AsmFlush;
            Move(s[1],OutBuf[OutCnt],length(s));
            inc(OutCnt,length(s));
            inc(AsmSize,length(s));
          end;
      end;


    Procedure TExternalAssemblerOutputFile.AsmWrite(const s:ansistring);
      begin
        if s='' then
          exit;
        if assigned(decorator) then
          AsmWriteFiltered(s)
        else
         AsmWriteAnsiStringUnfiltered(s);
      end;


    procedure TExternalAssemblerOutputFile.AsmWriteLn(const c: char);
      begin
        AsmWrite(c);
        AsmLn;
      end;


    Procedure TExternalAssemblerOutputFile.AsmWriteLn(const s:string);
      begin
        AsmWrite(s);
        AsmLn;
      end;


    Procedure TExternalAssemblerOutputFile.AsmWriteLn(const s: ansistring);
      begin
        AsmWrite(s);
        AsmLn;
      end;


    Procedure TExternalAssemblerOutputFile.AsmWritePChar(p:pchar);
      var
        i,j : longint;
      begin
        i:=StrLen(p);
        if i=0 then
          exit;
        if assigned(decorator) then
          AsmWriteFiltered(p,i)
        else
          begin
            j:=i;
            while j>0 do
             begin
               i:=min(j,AsmOutSize);
               if OutCnt+i>=AsmOutSize then
                AsmFlush;
               Move(p[0],OutBuf[OutCnt],i);
               inc(OutCnt,i);
               inc(AsmSize,i);
               dec(j,i);
               p:=pchar(@p[i]);
             end;
          end;
      end;


    Procedure TExternalAssemblerOutputFile.AsmLn;
      begin
        MaybeAddLinePostfix;
        if OutCnt>=AsmOutSize-2 then
         AsmFlush;
        if (cs_link_on_target in current_settings.globalswitches) then
          begin
            OutBuf[OutCnt]:=target_info.newline[1];
            inc(OutCnt);
            inc(AsmSize);
            if length(target_info.newline)>1 then
             begin
               OutBuf[OutCnt]:=target_info.newline[2];
               inc(OutCnt);
               inc(AsmSize);
             end;
          end
        else
          begin
            OutBuf[OutCnt]:=source_info.newline[1];
            inc(OutCnt);
            inc(AsmSize);
            if length(source_info.newline)>1 then
             begin
               OutBuf[OutCnt]:=source_info.newline[2];
               inc(OutCnt);
               inc(AsmSize);
             end;
          end;
      end;


    procedure TExternalAssemblerOutputFile.AsmCreate(Aplace:tcutplace);
{$ifdef hasamiga}
      var
        tempFileName: TPathStr;
{$endif}
      begin
        if owner.SmartAsm then
         owner.NextSmartName(Aplace);
{$ifdef hasamiga}
        { on Amiga/MorphOS try to redirect .s files to the T: assign, which is
          for temp files, and usually (default setting) located in the RAM: drive.
          This highly improves assembling speed for complex projects like the
          compiler itself, especially on hardware with slow disk I/O.
          Consider this as a poor man's pipe on Amiga, because real pipe handling
          would be much more complex and error prone to implement. (KB) }
        if (([cs_asm_extern,cs_asm_leave,cs_link_on_target] * current_settings.globalswitches) = []) then
         begin
          { try to have an unique name for the .s file }
          tempFileName:=HexStr(GetProcessID shr 4,7)+ExtractFileName(owner.AsmFileName);
{$ifndef morphos}
          { old Amiga RAM: handler only allows filenames up to 30 char }
          if Length(tempFileName) < 30 then
{$endif}
          owner.AsmFileName:='T:'+tempFileName;
         end;
{$endif}
{$ifdef hasunix}
        if owner.DoPipe then
         begin
           if owner.SmartAsm then
            begin
              if (owner.SmartFilesCount<=1) then
               Message1(exec_i_assembling_smart,owner.name);
            end
           else
             Message1(exec_i_assembling_pipe,owner.AsmFileName);
           if checkverbosity(V_Executable) then
             comment(V_Executable,'Executing "'+maybequoted(owner.FindAssembler)+'" with command line "'+
               owner.MakeCmdLine+'"');
           POpen(outfile,maybequoted(owner.FindAssembler)+' '+owner.MakeCmdLine,'W');
         end
        else
{$endif}
         begin
           Assign(outfile,owner.AsmFileName);
           {$push} {$I-}
           Rewrite(outfile,1);
           {$pop}
           if ioresult<>0 then
             begin
               fioerror:=true;
               Message1(exec_d_cant_create_asmfile,owner.AsmFileName);
             end;
         end;
        outcnt:=0;
        AsmSize:=0;
        AsmStartSize:=0;
      end;


    procedure TExternalAssemblerOutputFile.AsmClose;
      var
        f : file;
        FileAge : longint;
      begin
        AsmFlush;
{$ifdef hasunix}
        if owner.DoPipe then
          begin
            if PClose(outfile) <> 0 then
              GenerateError;
          end
        else
{$endif}
         begin
         {Touch Assembler time to ppu time is there is a ppufilename}
           if owner.ppufilename<>'' then
            begin
              Assign(f,owner.ppufilename);
              {$push} {$I-}
              reset(f,1);
              {$pop}
              if ioresult=0 then
               begin
                 FileAge := FileGetDate(GetFileHandle(f));
                 close(f);
                 reset(outfile,1);
                 FileSetDate(GetFileHandle(outFile),FileAge);
               end;
            end;
           close(outfile);
         end;
      end;

{*****************************************************************************
                                 TExternalAssembler
*****************************************************************************}


    function TExternalAssembler.single2str(d : single) : string;
      var
         hs : string;
      begin
         str(d,hs);
      { replace space with + }
         if hs[1]=' ' then
          hs[1]:='+';
         single2str:='0d'+hs
      end;

    function TExternalAssembler.double2str(d : double) : string;
      var
         hs : string;
      begin
         str(d,hs);
      { replace space with + }
         if hs[1]=' ' then
          hs[1]:='+';
         double2str:='0d'+hs
      end;

    function TExternalAssembler.extended2str(e : extended) : string;
      var
         hs : string;
      begin
         str(e,hs);
      { replace space with + }
         if hs[1]=' ' then
          hs[1]:='+';
         extended2str:='0d'+hs
      end;


    Function TExternalAssembler.DoPipe:boolean;
      begin
        DoPipe:=(cs_asm_pipe in current_settings.globalswitches) and
                (([cs_asm_extern,cs_asm_leave,cs_link_on_target] * current_settings.globalswitches) = []) and
                ((asminfo^.id in [as_gas,as_ggas,as_darwin,as_powerpc_xcoff,as_clang,as_solaris_as]));
      end;


    Constructor TExternalAssembler.Create(info: pasminfo; smart: boolean);
      begin
        inherited;
        if not assigned(fwriter) then
          begin
            fwriter:=TExternalAssemblerOutputFile.Create(self);
            ffreewriter:=true;
          end;
        if SmartAsm then
          begin
            path:=FixPath(ChangeFileExt(AsmFileName,target_info.smartext),false);
            CreateSmartLinkPath(path);
          end;
      end;


    constructor TExternalAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter,smart: boolean);
      begin
        fwriter:=wr;
        ffreewriter:=freewriter;
        Create(info,smart);
      end;


    procedure TExternalAssembler.CreateSmartLinkPath(const s:TPathStr);

        procedure DeleteFilesWithExt(const AExt:string);
        var
          dir : TRawByteSearchRec;
        begin
          if findfirst(FixPath(s,false)+'*'+AExt,faAnyFile,dir) = 0 then
            begin
              repeat
                DeleteFile(s+source_info.dirsep+dir.name);
              until findnext(dir) <> 0;
            end;
          findclose(dir);
        end;

      var
        hs  : TPathStr;
      begin
        if PathExists(s,false) then
         begin
           { the path exists, now we clean only all the .o and .s files }
           DeleteFilesWithExt(target_info.objext);
           DeleteFilesWithExt(target_info.asmext);
         end
        else
         begin
           hs:=s;
           if hs[length(hs)] in ['/','\'] then
            delete(hs,length(hs),1);
           {$push} {$I-}
            mkdir(hs);
           {$pop}
           if ioresult<>0 then;
         end;
      end;


    const
      lastas  : byte=255;
    var
      LastASBin : TCmdStr;
    Function TExternalAssembler.FindAssembler:string;
      var
        asfound : boolean;
        UtilExe  : string;
      begin
        asfound:=false;
        if cs_link_on_target in current_settings.globalswitches then
         begin
           { If linking on target, don't add any path PM }
           FindAssembler:=utilsprefix+ChangeFileExt(asminfo^.asmbin,target_info.exeext);
           exit;
         end
        else
         UtilExe:=utilsprefix+ChangeFileExt(asminfo^.asmbin,source_info.exeext);
        if lastas<>ord(asminfo^.id) then
         begin
           lastas:=ord(asminfo^.id);
           { is an assembler passed ? }
           if utilsdirectory<>'' then
             asfound:=FindFile(UtilExe,utilsdirectory,false,LastASBin);
           if not AsFound then
             asfound:=FindExe(UtilExe,false,LastASBin);
           if (not asfound) and not(cs_asm_extern in current_settings.globalswitches) then
            begin
              Message1(exec_e_assembler_not_found,LastASBin);
              current_settings.globalswitches:=current_settings.globalswitches+[cs_asm_extern];
            end;
           if asfound then
            Message1(exec_t_using_assembler,LastASBin);
         end;
        FindAssembler:=LastASBin;
      end;


    Function TExternalAssembler.CallAssembler(const command:string; const para:TCmdStr):Boolean;
      var
        DosExitCode : Integer;
      begin
        result:=true;
        if (cs_asm_extern in current_settings.globalswitches) then
          begin
            if SmartAsm then
              AsmRes.AddAsmCommand(command,para,Name+'('+TosTr(SmartFilesCount)+')')
            else
              AsmRes.AddAsmCommand(command,para,name);
            exit;
          end;
        try
          FlushOutput;
          DosExitCode:=RequotedExecuteProcess(command,para);
          if DosExitCode<>0
          then begin
            Message1(exec_e_error_while_assembling,tostr(dosexitcode));
            result:=false;
          end;
        except on E:EOSError do
          begin
            Message1(exec_e_cant_call_assembler,tostr(E.ErrorCode));
            current_settings.globalswitches:=current_settings.globalswitches+[cs_asm_extern];
            result:=false;
          end;
        end;
      end;


    Function TExternalAssembler.DoAssemble:boolean;
      begin
        DoAssemble:=true;
        if DoPipe then
         exit;
        if not(cs_asm_extern in current_settings.globalswitches) then
         begin
           if SmartAsm then
            begin
              if (SmartFilesCount<=1) then
               Message1(exec_i_assembling_smart,name);
            end
           else
           Message1(exec_i_assembling,name);
         end;

        if CallAssembler(FindAssembler,MakeCmdLine) then
         writer.RemoveAsm
        else
         begin
            DoAssemble:=false;
            GenerateError;
         end;
      end;


    function TExternalAssembler.MakeCmdLine: TCmdStr;
      begin
        result:=asminfo^.asmcmd;
        { for Xcode 7.x and later }
        if MacOSXVersionMin<>'' then
          Replace(result,'$DARWINVERSION','-mmacosx-version-min='+MacOSXVersionMin)
        else if iPhoneOSVersionMin<>'' then
          Replace(result,'$DARWINVERSION','-miphoneos-version-min='+iPhoneOSVersionMin)
        else
<<<<<<< HEAD
          Replace(result,'$DARWINVERSION','');
=======
          result:='-m68000 '+result;
{$endif}
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
{$ifdef arm}
        if (target_info.system=system_arm_darwin) then
          Replace(result,'$ARCH',lower(cputypestr[current_settings.cputype]));
{$endif arm}
        if (cs_link_on_target in current_settings.globalswitches) then
         begin
           Replace(result,'$ASM',maybequoted(ScriptFixFileName(AsmFileName)));
           Replace(result,'$OBJ',maybequoted(ScriptFixFileName(ObjFileName)));
         end
        else
         begin
{$ifdef hasunix}
          if DoPipe then
            if asminfo^.id<>as_clang then
              Replace(result,'$ASM','')
            else
              Replace(result,'$ASM','-')
          else
{$endif}
             Replace(result,'$ASM',maybequoted(AsmFileName));
           Replace(result,'$OBJ',maybequoted(ObjFileName));
         end;
         if (cs_create_pic in current_settings.moduleswitches) then
           Replace(result,'$PIC','-KPIC')
         else
           Replace(result,'$PIC','');
         if (cs_asm_source in current_settings.globalswitches) then
           Replace(result,'$NOWARN','')
         else
           Replace(result,'$NOWARN','-W');
         Replace(result,'$EXTRAOPT',asmextraopt);
      end;


    procedure TExternalAssembler.WriteSourceLine(hp: tailineinfo);
      var
        module : tmodule;
      begin
        { load infile }
        if (lastfileinfo.moduleindex<>hp.fileinfo.moduleindex) or
            (lastfileinfo.fileindex<>hp.fileinfo.fileindex) then
          begin
            { in case of a generic the module can be different }
            if current_module.unit_index=hp.fileinfo.moduleindex then
              module:=current_module
            else
              module:=get_module(hp.fileinfo.moduleindex);
            { during the compilation of the system unit there are cases when
              the fileinfo contains just zeros => invalid }
            if assigned(module) then
              infile:=module.sourcefiles.get_file(hp.fileinfo.fileindex)
            else
              infile:=nil;
            if assigned(infile) then
              begin
                { open only if needed !! }
                if (cs_asm_source in current_settings.globalswitches) then
                  infile.open;
              end;
            { avoid unnecessary reopens of the same file !! }
            lastfileinfo.fileindex:=hp.fileinfo.fileindex;
            lastfileinfo.moduleindex:=hp.fileinfo.moduleindex;
            { be sure to change line !! }
            lastfileinfo.line:=-1;
          end;
        { write source }
        if (cs_asm_source in current_settings.globalswitches) and
          assigned(infile) then
          begin
            if (infile<>lastinfile) then
              begin
                writer.AsmWriteLn(asminfo^.comment+'['+infile.name+']');
                if assigned(lastinfile) then
                  lastinfile.close;
              end;
            if (hp.fileinfo.line<>lastfileinfo.line) and
              (hp.fileinfo.line<infile.maxlinebuf) then
              begin
                if (hp.fileinfo.line<>0) and
                  (infile.linebuf^[hp.fileinfo.line]>=0) then
                  writer.AsmWriteLn(asminfo^.comment+'['+tostr(hp.fileinfo.line)+'] '+
                  fixline(infile.GetLineStr(hp.fileinfo.line)));
                { set it to a negative value !
                  to make that is has been read already !! PM }
                if (infile.linebuf^[hp.fileinfo.line]>=0) then
                  infile.linebuf^[hp.fileinfo.line]:=-infile.linebuf^[hp.fileinfo.line]-1;
              end;
          end;
        lastfileinfo:=hp.fileinfo;
        lastinfile:=infile;
      end;

    procedure TExternalAssembler.WriteTempalloc(hp: tai_tempalloc);
      begin
{$ifdef EXTDEBUG}
        if assigned(hp.problem) then
          writer.AsmWriteLn(asminfo^.comment+'Temp '+tostr(hp.temppos)+','+
          tostr(hp.tempsize)+' '+hp.problem^)
        else
{$endif EXTDEBUG}
          writer.AsmWriteLn(asminfo^.comment+'Temp '+tostr(hp.temppos)+','+
            tostr(hp.tempsize)+' '+tempallocstr[hp.allocation]);
      end;


    procedure TExternalAssembler.WriteRealConstAsBytes(hp: tai_realconst; const dbdir: string; do_line: boolean);
      var
        pdata: pbyte;
        index, step, swapmask, count: longint;
        ssingle: single;
        ddouble: double;
        ccomp: comp;
{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
        eextended: extended;
{$endif cpuextended}
      begin
        if do_line then
          begin
            case tai_realconst(hp).realtyp of
              aitrealconst_s32bit:
                writer.AsmWriteLn(asminfo^.comment+'value: '+single2str(tai_realconst(hp).value.s32val));
              aitrealconst_s64bit:
                writer.AsmWriteLn(asminfo^.comment+'value: '+double2str(tai_realconst(hp).value.s64val));
{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
              { can't write full 80 bit floating point constants yet on non-x86 }
              aitrealconst_s80bit:
                writer.AsmWriteLn(asminfo^.comment+'value: '+extended2str(tai_realconst(hp).value.s80val));
{$endif cpuextended}
              aitrealconst_s64comp:
                writer.AsmWriteLn(asminfo^.comment+'value: '+extended2str(tai_realconst(hp).value.s64compval));
              else
                internalerror(2014050604);
            end;
          end;
        writer.AsmWrite(dbdir);
        { generic float writing code: get start address of value, then write
          byte by byte. Can't use fields directly, because e.g ts64comp is
          defined as extended on x86 }
        case tai_realconst(hp).realtyp of
          aitrealconst_s32bit:
            begin
              ssingle:=single(tai_realconst(hp).value.s32val);
              pdata:=@ssingle;
            end;
          aitrealconst_s64bit:
            begin
              ddouble:=double(tai_realconst(hp).value.s64val);
              pdata:=@ddouble;
            end;
{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
          { can't write full 80 bit floating point constants yet on non-x86 }
          aitrealconst_s80bit:
            begin
              eextended:=extended(tai_realconst(hp).value.s80val);
              pdata:=@eextended;
            end;
{$endif cpuextended}
          aitrealconst_s64comp:
            begin
              ccomp:=comp(tai_realconst(hp).value.s64compval);
              pdata:=@ccomp;
            end;
          else
            internalerror(2014051001);
        end;
        count:=tai_realconst(hp).datasize;
        { write bytes in inverse order if source and target endianess don't
          match }
        if source_info.endian<>target_info.endian then
          begin
            { go from back to front }
            index:=count-1;
            step:=-1;
          end
        else
          begin
            index:=0;
            step:=1;
          end;
{$ifdef ARM}
        { ARM-specific: low and high dwords of a double may be swapped }
        if tai_realconst(hp).formatoptions=fo_hiloswapped then
          begin
            { only supported for double }
            if tai_realconst(hp).datasize<>8 then
              internalerror(2014050605);
            { switch bit of the index so that the words are written in
              the opposite order }
            swapmask:=4;
          end
        else
{$endif ARM}
          swapmask:=0;
        repeat
          writer.AsmWrite(tostr(pdata[index xor swapmask]));
          inc(index,step);
          dec(count);
          if count<>0 then
            writer.AsmWrite(',');
        until count=0;
        { padding }
        for count:=tai_realconst(hp).datasize+1 to tai_realconst(hp).savesize do
          writer.AsmWrite(',0');
        writer.AsmLn;
      end;


    procedure TExternalAssembler.WriteTree(p:TAsmList);
      begin
      end;


    procedure TExternalAssembler.WriteAsmList;
      begin
      end;


    procedure TExternalAssembler.MakeObject;
      begin
<<<<<<< HEAD
        writer.AsmCreate(cut_normal);
=======
        AsmCreate(cut_normal);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        FillChar(lastfileinfo, sizeof(lastfileinfo), 0);
        lastfileinfo.line := -1;
        lastinfile := nil;
        lastsectype := sec_none;
        WriteAsmList;
        writer.AsmClose;
        if not(writer.ioerror) then
          DoAssemble;
      end;


    destructor TExternalAssembler.Destroy;
      begin
        if ffreewriter then
          writer.Free;
        inherited;
      end;


{*****************************************************************************
                                  TInternalAssembler
*****************************************************************************}

    constructor TInternalAssembler.Create(info: pasminfo; smart: boolean);
      begin
        inherited;
        ObjOutput:=nil;
        ObjData:=nil;
        SmartAsm:=smart;
      end;


   destructor TInternalAssembler.destroy;
      begin
        if assigned(ObjData) then
          ObjData.free;
        if assigned(ObjOutput) then
          ObjOutput.free;
      end;


    procedure TInternalAssembler.WriteStab(p:pchar);

        function consumecomma(var p:pchar):boolean;
        begin
          while (p^=' ') do
            inc(p);
          result:=(p^=',');
          inc(p);
        end;

        function consumenumber(var p:pchar;out value:longint):boolean;
        var
          hs : string;
          len,
          code : integer;
        begin
          value:=0;
          while (p^=' ') do
            inc(p);
          len:=0;
          while (p^ in ['0'..'9']) do
            begin
              inc(len);
              hs[len]:=p^;
              inc(p);
            end;
          if len>0 then
            begin
              hs[0]:=chr(len);
              val(hs,value,code);
            end
          else
            code:=-1;
          result:=(code=0);
        end;

        function consumeoffset(var p:pchar;out relocsym:tobjsymbol;out value:longint):boolean;
        var
          hs        : string;
          len,
          code      : integer;
          pstart    : pchar;
          sym       : tobjsymbol;
          exprvalue : longint;
          gotmin,
          have_first_symbol,
          have_second_symbol,
          dosub     : boolean;
        begin
          result:=false;
          value:=0;
          relocsym:=nil;
          gotmin:=false;
          have_first_symbol:=false;
          have_second_symbol:=false;
          repeat
            dosub:=false;
            exprvalue:=0;
            if gotmin then
              begin
                dosub:=true;
                gotmin:=false;
              end;
            while (p^=' ') do
              inc(p);
            case p^ of
              #0 :
                break;
              ' ' :
                inc(p);
              '0'..'9' :
                begin
                  len:=0;
                  while (p^ in ['0'..'9']) do
                    begin
                      inc(len);
                      hs[len]:=p^;
                      inc(p);
                    end;
                  hs[0]:=chr(len);
                  val(hs,exprvalue,code);
                  if code<>0 then
                    internalerror(200702251);
                end;
              '.','_',
              'A'..'Z',
              'a'..'z' :
                begin
                  pstart:=p;
                  while not(p^ in [#0,' ','-','+']) do
                    inc(p);
                  len:=p-pstart;
                  if len>255 then
                    internalerror(200509187);
                  move(pstart^,hs[1],len);
                  hs[0]:=chr(len);
                  sym:=objdata.symbolref(hs);
                  have_first_symbol:=true;
                  { Second symbol? }
                  if assigned(relocsym) then
                    begin
                      if have_second_symbol then
                        internalerror(2007032201);
                      have_second_symbol:=true;
                      if not have_first_symbol then
                        internalerror(2007032202);
                      { second symbol should substracted to first }
                      if not dosub then
                        internalerror(2007032203);
                      if (relocsym.objsection<>sym.objsection) then
                        internalerror(2005091810);
                      exprvalue:=relocsym.address-sym.address;
                      relocsym:=nil;
                      dosub:=false;
                    end
                  else
                    begin
                      relocsym:=sym;
                      if assigned(sym.objsection) then
                        begin
                          { first symbol should be + }
                          if not have_first_symbol and dosub then
                            internalerror(2007032204);
                          have_first_symbol:=true;
                        end;
                    end;
                end;
              '+' :
                begin
                  { nothing, by default addition is done }
                  inc(p);
                end;
              '-' :
                begin
                  gotmin:=true;
                  inc(p);
                end;
              else
                internalerror(200509189);
            end;
            if dosub then
              dec(value,exprvalue)
            else
              inc(value,exprvalue);
          until false;
          result:=true;
        end;

      var
        stabstrlen,
        ofs,
        nline,
        nidx,
        nother,
        i         : longint;
        stab      : TObjStabEntry;
        relocsym  : TObjSymbol;
        pstr,
        pcurr,
        pendquote : pchar;
        oldsec    : TObjSection;
      begin
        pcurr:=nil;
        pstr:=nil;
        pendquote:=nil;
        relocsym:=nil;
        ofs:=0;

        { Parse string part }
        if (p[0]='"') then
          begin
            pstr:=@p[1];
            { Ignore \" inside the string }
            i:=1;
            while not((p[i]='"') and (p[i-1]<>'\')) and
                  (p[i]<>#0) do
              inc(i);
            pendquote:=@p[i];
            pendquote^:=#0;
            pcurr:=@p[i+1];
            if not consumecomma(pcurr) then
              internalerror(200509181);
          end
        else
          pcurr:=p;

        { When in pass 1 then only alloc and leave }
        if ObjData.currpass=1 then
          begin
            ObjData.StabsSec.Alloc(sizeof(TObjStabEntry));
            if assigned(pstr) and (pstr[0]<>#0) then
              ObjData.StabStrSec.Alloc(strlen(pstr)+1);
          end
        else
          begin
            { Stabs format: nidx,nother,nline[,offset] }
            if not consumenumber(pcurr,nidx) then
              internalerror(200509182);
            if not consumecomma(pcurr) then
              internalerror(200509183);
            if not consumenumber(pcurr,nother) then
              internalerror(200509184);
            if not consumecomma(pcurr) then
              internalerror(200509185);
            if not consumenumber(pcurr,nline) then
              internalerror(200509186);
            if consumecomma(pcurr) then
              consumeoffset(pcurr,relocsym,ofs);

            { Generate stab entry }
            if assigned(pstr) and (pstr[0]<>#0) then
              begin
                stabstrlen:=strlen(pstr);
{$ifdef optimizestabs}
                StabStrEntry:=nil;
                if (nidx=N_SourceFile) or (nidx=N_IncludeFile) then
                  begin
                    hs:=strpas(pstr);
                    StabstrEntry:=StabStrDict.Find(hs);
                    if not assigned(StabstrEntry) then
                      begin
                        StabstrEntry:=TStabStrEntry.Create(hs);
                        StabstrEntry:=StabStrSec.Size;
                        StabStrDict.Insert(StabstrEntry);
                        { generate new stab }
                        StabstrEntry:=nil;
                      end;
                  end;
                if assigned(StabstrEntry) then
                  stab.strpos:=StabstrEntry.strpos
                else
{$endif optimizestabs}
                  begin
                    stab.strpos:=ObjData.StabStrSec.Size;
                    ObjData.StabStrSec.write(pstr^,stabstrlen+1);
                  end;
              end
            else
              stab.strpos:=0;
            stab.ntype:=byte(nidx);
            stab.ndesc:=word(nline);
            stab.nother:=byte(nother);
            stab.nvalue:=ofs;

            { Write the stab first without the value field. Then
              write a the value field with relocation }
            oldsec:=ObjData.CurrObjSec;
            ObjData.SetSection(ObjData.StabsSec);
            ObjData.Writebytes(stab,sizeof(TObjStabEntry)-4);
<<<<<<< HEAD
<<<<<<< HEAD
            ObjData.Writereloc(stab.nvalue,4,relocsym,RELOC_ABSOLUTE32);
=======
            ObjData.Writereloc(stab.nvalue,4,relocsym,RELOC_ABSOLUTE);
>>>>>>> graemeg/fixes_2_2
=======
            ObjData.Writereloc(stab.nvalue,4,relocsym,RELOC_ABSOLUTE);
>>>>>>> origin/fixes_2_2
            ObjData.setsection(oldsec);
          end;
        if assigned(pendquote) then
          pendquote^:='"';
      end;


    function TInternalAssembler.MaybeNextList(var hp:Tai):boolean;
      begin
        { maybe end of list }
        while not assigned(hp) do
         begin
           if currlistidx<lists then
            begin
              inc(currlistidx);
              currlist:=list[currlistidx];
              hp:=Tai(currList.first);
            end
           else
            begin
              MaybeNextList:=false;
              exit;
            end;
         end;
        MaybeNextList:=true;
      end;


    function TInternalAssembler.SetIndirectToSymbol(hp: Tai; const indirectname: string): Boolean;
      var
        objsym  : TObjSymbol;
        indsym  : TObjSymbol;
      begin
        Result:=
          Assigned(hp) and
          (hp.typ=ait_symbol);
        if not Result then
          Exit;
        objsym:=Objdata.SymbolRef(tai_symbol(hp).sym);
        objsym.size:=0;

        indsym := TObjSymbol(ObjData.ObjSymbolList.Find(indirectname));
        if not Assigned(indsym) then
          begin
            { it's possible that indirect symbol is not present in the list,
              so we must create it as undefined }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            indsym:=ObjData.CObjSymbol.Create(ObjData.ObjSymbolList, indirectname);
=======
            indsym:=TObjSymbol.Create(ObjData.ObjSymbolList, indirectname);
>>>>>>> graemeg/cpstrnew
=======
            indsym:=TObjSymbol.Create(ObjData.ObjSymbolList, indirectname);
>>>>>>> graemeg/cpstrnew
=======
            indsym:=TObjSymbol.Create(ObjData.ObjSymbolList, indirectname);
>>>>>>> graemeg/cpstrnew
=======
            indsym:=TObjSymbol.Create(ObjData.ObjSymbolList, indirectname);
>>>>>>> origin/cpstrnew
            indsym.typ:=AT_NONE;
            indsym.bind:=AB_NONE;
          end;
        objsym.indsymbol:=indsym;
        Result:=true;
      end;


    function TInternalAssembler.TreePass0(hp:Tai):Tai;
      var
        objsym,
        objsymend : TObjSymbol;
      begin
        while assigned(hp) do
         begin
           case hp.typ of
             ait_align :
               begin
                 if tai_align_abstract(hp).aligntype>1 then
                   begin
                     { always use the maximum fillsize in this pass to avoid possible
                       short jumps to become out of range }
                     Tai_align_abstract(hp).fillsize:=Tai_align_abstract(hp).aligntype;
                     ObjData.alloc(Tai_align_abstract(hp).fillsize);
<<<<<<< HEAD
<<<<<<< HEAD
                     { may need to increase alignment of section }
                     if tai_align_abstract(hp).aligntype>ObjData.CurrObjSec.secalign then
                       ObjData.CurrObjSec.secalign:=tai_align_abstract(hp).aligntype;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                   end
                 else
                   Tai_align_abstract(hp).fillsize:=0;
               end;
             ait_datablock :
               begin
{$ifdef USE_COMM_IN_BSS}
                 if writingpackages and
                    Tai_datablock(hp).is_global then
                   ObjData.SymbolDefine(Tai_datablock(hp).sym)
                 else
{$endif USE_COMM_IN_BSS}
                   begin
                     ObjData.allocalign(used_align(size_2_align(Tai_datablock(hp).size),0,ObjData.CurrObjSec.secalign));
                     ObjData.SymbolDefine(Tai_datablock(hp).sym);
                     ObjData.alloc(Tai_datablock(hp).size);
                   end;
               end;
<<<<<<< HEAD
             ait_realconst:
               ObjData.alloc(tai_realconst(hp).savesize);
=======
             ait_real_80bit :
               ObjData.alloc(tai_real_80bit(hp).savesize);
             ait_real_64bit :
               ObjData.alloc(8);
             ait_real_32bit :
               ObjData.alloc(4);
             ait_comp_64bit :
               ObjData.alloc(8);
>>>>>>> graemeg/cpstrnew
             ait_const:
               begin
                 { if symbols are provided we can calculate the value for relative symbols.
                   This is required for length calculation of leb128 constants }
                 if assigned(tai_const(hp).sym) then
                   begin
                     objsym:=Objdata.SymbolRef(tai_const(hp).sym);
<<<<<<< HEAD
<<<<<<< HEAD
                     { objsym already defined and there is endsym? }
                     if assigned(objsym.objsection) and assigned(tai_const(hp).endsym) then
                       begin
                         objsymend:=Objdata.SymbolRef(tai_const(hp).endsym);
                         { objsymend already defined? }
                         if assigned(objsymend.objsection) then
                           begin
                             if objsymend.objsection<>objsym.objsection then
                               begin
                                 { leb128 relative constants are not relocatable, but other types are,
                                   given that objsym belongs to the current section. }
                                 if (Tai_const(hp).consttype in [aitconst_uleb128bit,aitconst_sleb128bit]) or
                                    (objsym.objsection<>ObjData.CurrObjSec) then
                                   InternalError(200404124);
                               end
                             else
                               Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
                           end;
=======
=======
>>>>>>> origin/fixes_2_2
                     if assigned(tai_const(hp).endsym) then
                       begin
                         objsymend:=Objdata.SymbolRef(tai_const(hp).endsym);
                         if objsymend.objsection<>objsym.objsection then
                           internalerror(200404124);
                         Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                       end;
                   end;
                 ObjData.alloc(tai_const(hp).size);
               end;
<<<<<<< HEAD
<<<<<<< HEAD
             ait_directive:
               begin
                 case tai_directive(hp).directive of
                   asd_indirect_symbol:
                     { handled in TreePass1 }
                     ;
                   asd_lazy_reference:
                     begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       if tai_directive(hp).name='' then
                         Internalerror(2009112101);
                       objsym:=ObjData.symbolref(tai_directive(hp).name);
=======
                       if tai_directive(hp).name = nil then
                         Internalerror(2009112101);
                       objsym:=ObjData.symbolref(tai_directive(hp).name^);
>>>>>>> graemeg/cpstrnew
=======
                       if tai_directive(hp).name = nil then
                         Internalerror(2009112101);
                       objsym:=ObjData.symbolref(tai_directive(hp).name^);
>>>>>>> graemeg/cpstrnew
=======
                       if tai_directive(hp).name = nil then
                         Internalerror(2009112101);
                       objsym:=ObjData.symbolref(tai_directive(hp).name^);
>>>>>>> graemeg/cpstrnew
=======
                       if tai_directive(hp).name = nil then
                         Internalerror(2009112101);
                       objsym:=ObjData.symbolref(tai_directive(hp).name^);
>>>>>>> origin/cpstrnew
                       objsym.bind:=AB_LAZY;
                     end;
                   asd_reference:
                     { ignore for now, but should be added}
                     ;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef ARM}
                   asd_thumb_func:
                     ObjData.ThumbFunc:=true;
{$endif ARM}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                   else
                     internalerror(2010011101);
                 end;
               end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             ait_section:
               begin
                 ObjData.CreateSection(Tai_section(hp).sectype,Tai_section(hp).name^,Tai_section(hp).secorder);
                 Tai_section(hp).sec:=ObjData.CurrObjSec;
               end;
             ait_symbol :
               begin
                 { needs extra support in the internal assembler }
                 { the value is just ignored }
                 {if tai_symbol(hp).has_value then
                      internalerror(2009090804); ;}
                 ObjData.SymbolDefine(Tai_symbol(hp).sym);
               end;
             ait_label :
               ObjData.SymbolDefine(Tai_label(hp).labsym);
             ait_string :
               ObjData.alloc(Tai_string(hp).len);
             ait_instruction :
               begin
                 { reset instructions which could change in pass 2 }
                 Taicpu(hp).resetpass2;
                 ObjData.alloc(Taicpu(hp).Pass1(ObjData));
               end;
             ait_cutobject :
               if SmartAsm then
                break;
           end;
           hp:=Tai(hp.next);
         end;
        TreePass0:=hp;
      end;


    function TInternalAssembler.TreePass1(hp:Tai):Tai;
      var
        objsym,
        objsymend : TObjSymbol;
      begin
        while assigned(hp) do
         begin
           case hp.typ of
             ait_align :
               begin
                 if tai_align_abstract(hp).aligntype>1 then
                   begin
                     { here we must determine the fillsize which is used in pass2 }
                     Tai_align_abstract(hp).fillsize:=align(ObjData.CurrObjSec.Size,Tai_align_abstract(hp).aligntype)-
                       ObjData.CurrObjSec.Size;
                     ObjData.alloc(Tai_align_abstract(hp).fillsize);
                   end;
               end;
             ait_datablock :
               begin
                 if (oso_data in ObjData.CurrObjSec.secoptions) and
                    not (oso_sparse_data in ObjData.CurrObjSec.secoptions) then
                   Message(asmw_e_alloc_data_only_in_bss);
{$ifdef USE_COMM_IN_BSS}
                 if writingpackages and
                    Tai_datablock(hp).is_global then
                   begin
                     objsym:=ObjData.SymbolDefine(Tai_datablock(hp).sym);
                     objsym.size:=Tai_datablock(hp).size;
                     objsym.bind:=AB_COMMON;
                     objsym.alignment:=needtowritealignmentalsoforELF;
                   end
                 else
{$endif USE_COMM_IN_BSS}
                   begin
                     ObjData.allocalign(used_align(size_2_align(Tai_datablock(hp).size),0,ObjData.CurrObjSec.secalign));
                     objsym:=ObjData.SymbolDefine(Tai_datablock(hp).sym);
                     objsym.size:=Tai_datablock(hp).size;
                     ObjData.alloc(Tai_datablock(hp).size);
                   end;
               end;
<<<<<<< HEAD
             ait_realconst:
               ObjData.alloc(tai_realconst(hp).savesize);
=======
             ait_real_80bit :
               ObjData.alloc(tai_real_80bit(hp).savesize);
             ait_real_64bit :
               ObjData.alloc(8);
             ait_real_32bit :
               ObjData.alloc(4);
             ait_comp_64bit :
               ObjData.alloc(8);
>>>>>>> graemeg/cpstrnew
             ait_const:
               begin
<<<<<<< HEAD
<<<<<<< HEAD
                 { Recalculate relative symbols }
=======
                 { Recalculate relative symbols, all checks are done in treepass0 }
>>>>>>> graemeg/fixes_2_2
=======
                 { Recalculate relative symbols, all checks are done in treepass0 }
>>>>>>> origin/fixes_2_2
                 if assigned(tai_const(hp).sym) and
                    assigned(tai_const(hp).endsym) then
                   begin
                     objsym:=Objdata.SymbolRef(tai_const(hp).sym);
                     objsymend:=Objdata.SymbolRef(tai_const(hp).endsym);
<<<<<<< HEAD
<<<<<<< HEAD
                     if objsymend.objsection<>objsym.objsection then
                       begin
                         if (Tai_const(hp).consttype in [aitconst_uleb128bit,aitconst_sleb128bit]) or
                            (objsym.objsection<>ObjData.CurrObjSec) then
                           internalerror(200905042);
                       end
                     else
                       Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
=======
                     Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
>>>>>>> graemeg/fixes_2_2
=======
                     Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
>>>>>>> origin/fixes_2_2
                   end;
                 ObjData.alloc(tai_const(hp).size);
               end;
             ait_section:
               begin
                 { use cached value }
                 ObjData.setsection(Tai_section(hp).sec);
               end;
             ait_stab :
               begin
                 if assigned(Tai_stab(hp).str) then
                   WriteStab(Tai_stab(hp).str);
               end;
             ait_symbol :
               ObjData.SymbolDefine(Tai_symbol(hp).sym);
             ait_symbol_end :
               begin
                 objsym:=ObjData.SymbolRef(Tai_symbol_end(hp).sym);
                 objsym.size:=ObjData.CurrObjSec.Size-objsym.offset;
               end;
             ait_label :
               ObjData.SymbolDefine(Tai_label(hp).labsym);
             ait_string :
               ObjData.alloc(Tai_string(hp).len);
             ait_instruction :
               ObjData.alloc(Taicpu(hp).Pass1(ObjData));
             ait_cutobject :
               if SmartAsm then
                break;
<<<<<<< HEAD
<<<<<<< HEAD
             ait_directive :
               begin
                 case tai_directive(hp).directive of
                   asd_indirect_symbol:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     if tai_directive(hp).name='' then
                       Internalerror(2009101103)
                     else if not SetIndirectToSymbol(Tai(hp.Previous), tai_directive(hp).name) then
=======
                     if tai_directive(hp).name = nil then
                       Internalerror(2009101103)
                     else if not SetIndirectToSymbol(Tai(hp.Previous), tai_directive(hp).name^) then
>>>>>>> graemeg/cpstrnew
=======
                     if tai_directive(hp).name = nil then
                       Internalerror(2009101103)
                     else if not SetIndirectToSymbol(Tai(hp.Previous), tai_directive(hp).name^) then
>>>>>>> graemeg/cpstrnew
=======
                     if tai_directive(hp).name = nil then
                       Internalerror(2009101103)
                     else if not SetIndirectToSymbol(Tai(hp.Previous), tai_directive(hp).name^) then
>>>>>>> graemeg/cpstrnew
=======
                     if tai_directive(hp).name = nil then
                       Internalerror(2009101103)
                     else if not SetIndirectToSymbol(Tai(hp.Previous), tai_directive(hp).name^) then
>>>>>>> origin/cpstrnew
                       Internalerror(2009101102);
                   asd_lazy_reference:
                     { handled in TreePass0 }
                     ;
                   asd_reference:
                     { ignore for now, but should be added}
                     ;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   asd_thumb_func:
                     { ignore for now, but should be added}
                     ;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                   else
                     internalerror(2010011102);
                 end;
               end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           end;
           hp:=Tai(hp.next);
         end;
        TreePass1:=hp;
      end;


    function TInternalAssembler.TreePass2(hp:Tai):Tai;
      var
        fillbuffer : tfillbuffer;
<<<<<<< HEAD
<<<<<<< HEAD
        leblen : byte;
        lebbuf : array[0..63] of byte;
        objsym,
        ref,
        objsymend : TObjSymbol;
        zerobuf : array[0..63] of byte;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        relative_reloc: boolean;
        pdata : pointer;
        ssingle : single;
        ddouble : double;
        {$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
        eextended : extended;
        {$endif}
        ccomp : comp;
        tmp    : word;
      begin
        fillchar(zerobuf,sizeof(zerobuf),0);
        fillchar(objsym,sizeof(objsym),0);
        fillchar(objsymend,sizeof(objsymend),0);
=======
      begin
        fillchar(zerobuf,sizeof(zerobuf),0);
>>>>>>> graemeg/cpstrnew
=======
      begin
        fillchar(zerobuf,sizeof(zerobuf),0);
>>>>>>> graemeg/cpstrnew
=======
      begin
        fillchar(zerobuf,sizeof(zerobuf),0);
>>>>>>> graemeg/cpstrnew
=======
      begin
        fillchar(zerobuf,sizeof(zerobuf),0);
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef x86}
        co : comp;
{$endif x86}
        leblen : byte;
        lebbuf : array[0..63] of byte;
        objsym,
        objsymend : TObjSymbol;
      begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        { main loop }
        while assigned(hp) do
         begin
           case hp.typ of
             ait_align :
               begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                 if tai_align_abstract(hp).aligntype>ObjData.CurrObjSec.secalign then
                   InternalError(2012072301);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                 if oso_data in ObjData.CurrObjSec.secoptions then
                   ObjData.writebytes(Tai_align_abstract(hp).calculatefillbuf(fillbuffer,oso_executable in ObjData.CurrObjSec.secoptions)^,
                     Tai_align_abstract(hp).fillsize)
                 else
                   ObjData.alloc(Tai_align_abstract(hp).fillsize);
               end;
             ait_section :
               begin
                 { use cached value }
                 ObjData.setsection(Tai_section(hp).sec);
               end;
             ait_symbol :
               begin
                 ObjOutput.exportsymbol(ObjData.SymbolRef(Tai_symbol(hp).sym));
               end;
            ait_symbol_end :
               begin
                 { recalculate size, as some preceding instructions
                   could have been changed to smaller size }
                 objsym:=ObjData.SymbolRef(Tai_symbol_end(hp).sym);
                 objsym.size:=ObjData.CurrObjSec.Size-objsym.offset;
               end;
             ait_datablock :
               begin
                 ObjOutput.exportsymbol(ObjData.SymbolRef(Tai_datablock(hp).sym));
{$ifdef USE_COMM_IN_BSS}
                 if not(writingpackages and
                        Tai_datablock(hp).is_global) then
{$endif USE_COMM_IN_BSS}
                   begin
                     ObjData.allocalign(used_align(size_2_align(Tai_datablock(hp).size),0,ObjData.CurrObjSec.secalign));
                     ObjData.alloc(Tai_datablock(hp).size);
                   end;
               end;
<<<<<<< HEAD
             ait_realconst:
=======
             ait_real_80bit :
               begin
                 ObjData.writebytes(Tai_real_80bit(hp).value,10);
                 ObjData.writebytes(zerobuf,Tai_real_80bit(hp).savesize-10);
               end;
             ait_real_64bit :
               ObjData.writebytes(Tai_real_64bit(hp).value,8);
             ait_real_32bit :
               ObjData.writebytes(Tai_real_32bit(hp).value,4);
             ait_comp_64bit :
>>>>>>> graemeg/cpstrnew
               begin
                 case tai_realconst(hp).realtyp of
                   aitrealconst_s32bit:
                     begin
                       ssingle:=single(tai_realconst(hp).value.s32val);
                       pdata:=@ssingle;
                     end;
                   aitrealconst_s64bit:
                     begin
                       ddouble:=double(tai_realconst(hp).value.s64val);
                       pdata:=@ddouble;
                     end;
         {$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
                   { can't write full 80 bit floating point constants yet on non-x86 }
                   aitrealconst_s80bit:
                     begin
                       eextended:=extended(tai_realconst(hp).value.s80val);
                       pdata:=@eextended;
                     end;
         {$endif cpuextended}
                   aitrealconst_s64comp:
                     begin
                       ccomp:=comp(tai_realconst(hp).value.s64compval);
                       pdata:=@ccomp;
                     end;
                   else
                     internalerror(2015030501);
                 end;
                 ObjData.writebytes(pdata^,tai_realconst(hp).datasize);
                 ObjData.writebytes(zerobuf,tai_realconst(hp).savesize-tai_realconst(hp).datasize);
               end;
             ait_string :
               ObjData.writebytes(Tai_string(hp).str^,Tai_string(hp).len);
             ait_const :
               begin
                 { Recalculate relative symbols, addresses of forward references
                   can be changed in treepass1 }
<<<<<<< HEAD
<<<<<<< HEAD
                 relative_reloc:=false;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                 if assigned(tai_const(hp).sym) and
                    assigned(tai_const(hp).endsym) then
                   begin
                     objsym:=Objdata.SymbolRef(tai_const(hp).sym);
                     objsymend:=Objdata.SymbolRef(tai_const(hp).endsym);
<<<<<<< HEAD
<<<<<<< HEAD
                     relative_reloc:=(objsym.objsection<>objsymend.objsection);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                     Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
                   end;
                 case tai_const(hp).consttype of
                   aitconst_64bit,
                   aitconst_32bit,
                   aitconst_16bit,
                   aitconst_64bit_unaligned,
                   aitconst_32bit_unaligned,
                   aitconst_16bit_unaligned,
                   aitconst_8bit :
                     begin
                       if assigned(tai_const(hp).sym) and
                          not assigned(tai_const(hp).endsym) then
                         ObjData.writereloc(Tai_const(hp).symofs,tai_const(hp).size,Objdata.SymbolRef(tai_const(hp).sym),RELOC_ABSOLUTE)
<<<<<<< HEAD
<<<<<<< HEAD
                       else if relative_reloc then
                         ObjData.writereloc(ObjData.CurrObjSec.size+tai_const(hp).size-objsym.address+tai_const(hp).symofs,tai_const(hp).size,objsymend,RELOC_RELATIVE)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                       else
                         ObjData.writebytes(Tai_const(hp).value,tai_const(hp).size);
                     end;
                   aitconst_rva_symbol :
                     begin
                       { PE32+? }
                       if target_info.system=system_x86_64_win64 then
                         ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_RVA)
                       else
<<<<<<< HEAD
<<<<<<< HEAD
                         ObjData.writereloc(Tai_const(hp).symofs,sizeof(pint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_RVA);
=======
                         ObjData.writereloc(Tai_const(hp).symofs,sizeof(aint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_RVA);
>>>>>>> graemeg/fixes_2_2
=======
                         ObjData.writereloc(Tai_const(hp).symofs,sizeof(aint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_RVA);
>>>>>>> origin/fixes_2_2
                     end;
                   aitconst_secrel32_symbol :
                     begin
                       { Required for DWARF2 support under Windows }
                       ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_SECREL32);
                     end;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef i8086}
                   aitconst_farptr :
                     if assigned(tai_const(hp).sym) and
                        not assigned(tai_const(hp).endsym) then
                       ObjData.writereloc(Tai_const(hp).symofs,tai_const(hp).size,Objdata.SymbolRef(tai_const(hp).sym),RELOC_FARPTR)
                     else if relative_reloc then
                       internalerror(2015040601)
                     else
                       ObjData.writebytes(Tai_const(hp).value,tai_const(hp).size);
                   aitconst_seg:
                     if assigned(tai_const(hp).sym) and (tai_const(hp).size=2) then
                       ObjData.writereloc(0,2,Objdata.SymbolRef(tai_const(hp).sym),RELOC_SEG)
                     else
                       internalerror(2015110502);
                   aitconst_dgroup:
                     ObjData.writereloc(0,2,nil,RELOC_DGROUP);
                   aitconst_fardataseg:
                     ObjData.writereloc(0,2,nil,RELOC_FARDATASEG);
{$endif i8086}
{$ifdef arm}
                   aitconst_got:
                     ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_GOT32);
{$endif arm}
                   aitconst_gotoff_symbol:
                     ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_GOTOFF);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                   aitconst_uleb128bit,
                   aitconst_sleb128bit :
                     begin
                       if tai_const(hp).consttype=aitconst_uleb128bit then
                         leblen:=EncodeUleb128(qword(Tai_const(hp).value),lebbuf)
                       else
                         leblen:=EncodeSleb128(Tai_const(hp).value,lebbuf);
                       if leblen<>tai_const(hp).size then
                         internalerror(200709271);
                       ObjData.writebytes(lebbuf,leblen);
                     end;
                   aitconst_darwin_dwarf_delta32,
                   aitconst_darwin_dwarf_delta64:
                     ObjData.writebytes(Tai_const(hp).value,tai_const(hp).size);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   aitconst_half16bit,
                   aitconst_gs:
                     begin
                       tmp:=Tai_const(hp).value div 2;
                       ObjData.writebytes(tmp,2);
                     end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                   else
                     internalerror(200603254);
                 end;
               end;
             ait_label :
               begin
                 { exporting shouldn't be necessary as labels are local,
                   but it's better to be on the safe side (PFV) }
                 ObjOutput.exportsymbol(ObjData.SymbolRef(Tai_label(hp).labsym));
               end;
             ait_instruction :
               Taicpu(hp).Pass2(ObjData);
             ait_stab :
               WriteStab(Tai_stab(hp).str);
             ait_function_name,
             ait_force_line : ;
             ait_cutobject :
               if SmartAsm then
                break;
<<<<<<< HEAD
<<<<<<< HEAD
             ait_weak:
               begin
                 objsym:=ObjData.symbolref(tai_weak(hp).sym^);
                 objsym.bind:=AB_WEAK_EXTERNAL;
               end;
             ait_symbolpair:
               begin
                 if tai_symbolpair(hp).kind=spk_set then
                   begin
                     objsym:=ObjData.symbolref(tai_symbolpair(hp).sym^);
                     ref:=objdata.symbolref(tai_symbolpair(hp).value^);

                     objsym.offset:=ref.offset;
                     objsym.objsection:=ref.objsection;
{$ifdef arm}
                     objsym.ThumbFunc:=ref.ThumbFunc;
{$endif arm}
                   end;
               end;
{$ifndef DISABLE_WIN64_SEH}
             ait_seh_directive :
               tai_seh_directive(hp).generate_code(objdata);
{$endif DISABLE_WIN64_SEH}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           end;
           hp:=Tai(hp.next);
         end;
        TreePass2:=hp;
      end;


    procedure TInternalAssembler.writetree;
      label
        doexit;
      var
        hp : Tai;
        ObjWriter : TObjectWriter;
      begin
        ObjWriter:=TObjectwriter.create;
        ObjOutput:=CObjOutput.Create(ObjWriter);
        ObjData:=ObjOutput.newObjData(ObjFileName);

        { Pass 0 }
        ObjData.currpass:=0;
        ObjData.createsection(sec_code);
        ObjData.beforealloc;
        { start with list 1 }
        currlistidx:=1;
        currlist:=list[currlistidx];
        hp:=Tai(currList.first);
        while assigned(hp) do
         begin
           hp:=TreePass0(hp);
           MaybeNextList(hp);
         end;
        ObjData.afteralloc;
        { leave if errors have occured }
        if errorcount>0 then
         goto doexit;

        { Pass 1 }
        ObjData.currpass:=1;
        ObjData.resetsections;
        ObjData.beforealloc;
        ObjData.createsection(sec_code);
        { start with list 1 }
        currlistidx:=1;
        currlist:=list[currlistidx];
        hp:=Tai(currList.first);
        while assigned(hp) do
         begin
           hp:=TreePass1(hp);
           MaybeNextList(hp);
         end;
        ObjData.createsection(sec_code);
        ObjData.afteralloc;

        { leave if errors have occured }
        if errorcount>0 then
         goto doexit;

        { Pass 2 }
        ObjData.currpass:=2;
        ObjData.resetsections;
        ObjData.beforewrite;
        ObjData.createsection(sec_code);
        { start with list 1 }
        currlistidx:=1;
        currlist:=list[currlistidx];
        hp:=Tai(currList.first);
        while assigned(hp) do
         begin
           hp:=TreePass2(hp);
           MaybeNextList(hp);
         end;
        ObjData.createsection(sec_code);
        ObjData.afterwrite;

        { don't write the .o file if errors have occured }
        if errorcount=0 then
         begin
           { write objectfile }
           ObjOutput.startobjectfile(ObjFileName);
           ObjOutput.writeobjectfile(ObjData);
         end;

      doexit:
        { Cleanup }
        ObjData.free;
        ObjData:=nil;
        ObjWriter.free;
      end;


    procedure TInternalAssembler.writetreesmart;
      var
        hp : Tai;
        startsectype : TAsmSectiontype;
        place: tcutplace;
        ObjWriter : TObjectWriter;
        startsecname: String;
        startsecorder: TAsmSectionOrder;
      begin
        if not(cs_asm_leave in current_settings.globalswitches) and
           not(af_needar in asminfo^.flags) then
          ObjWriter:=CInternalAr.CreateAr(current_module.staticlibfilename)
        else
          ObjWriter:=TObjectwriter.create;

        NextSmartName(cut_normal);
        ObjOutput:=CObjOutput.Create(ObjWriter);
        startsectype:=sec_none;
        startsecname:='';
        startsecorder:=secorder_default;

        { start with list 1 }
        currlistidx:=1;
        currlist:=list[currlistidx];
        hp:=Tai(currList.first);
        while assigned(hp) do
         begin
           ObjData:=ObjOutput.newObjData(ObjFileName);

           { Pass 0 }
           ObjData.currpass:=0;
           ObjData.resetsections;
           ObjData.beforealloc;
           if startsectype<>sec_none then
             ObjData.CreateSection(startsectype,startsecname,startsecorder);
           TreePass0(hp);
           ObjData.afteralloc;
           { leave if errors have occured }
           if errorcount>0 then
             break;

           { Pass 1 }
           ObjData.currpass:=1;
           ObjData.resetsections;
           ObjData.beforealloc;
           if startsectype<>sec_none then
             ObjData.CreateSection(startsectype,startsecname,startsecorder);
           TreePass1(hp);
           ObjData.afteralloc;

           { leave if errors have occured }
           if errorcount>0 then
             break;

           { Pass 2 }
           ObjData.currpass:=2;
           ObjOutput.startobjectfile(ObjFileName);
           ObjData.resetsections;
           ObjData.beforewrite;
           if startsectype<>sec_none then
             ObjData.CreateSection(startsectype,startsecname,startsecorder);
           hp:=TreePass2(hp);
           ObjData.afterwrite;

           { leave if errors have occured }
           if errorcount>0 then
             break;

           { write the current objectfile }
           ObjOutput.writeobjectfile(ObjData);
           ObjData.free;
           ObjData:=nil;

           { end of lists? }
           if not MaybeNextList(hp) then
             break;

           { we will start a new objectfile so reset everything }
           { The place can still change in the next while loop, so don't init }
           { the writer yet (JM)                                              }
           if (hp.typ=ait_cutobject) then
             place := Tai_cutobject(hp).place
           else
             place := cut_normal;

           { avoid empty files }
           startsectype:=sec_none;
           startsecname:='';
           startsecorder:=secorder_default;
           while assigned(hp) and
                 (Tai(hp).typ in [ait_marker,ait_comment,ait_section,ait_cutobject]) do
            begin
              if Tai(hp).typ=ait_section then
                begin
                  startsectype:=Tai_section(hp).sectype;
                  startsecname:=Tai_section(hp).name^;
                  startsecorder:=Tai_section(hp).secorder;
                end;
              if (Tai(hp).typ=ait_cutobject) then
                place:=Tai_cutobject(hp).place;
              hp:=Tai(hp.next);
            end;

           if not MaybeNextList(hp) then
             break;

           { start next objectfile }
           NextSmartName(place);
         end;
        ObjData.free;
        ObjData:=nil;
        ObjWriter.free;
      end;


    procedure TInternalAssembler.MakeObject;

    var to_do:set of TasmlistType;
        i:TasmlistType;

        procedure addlist(p:TAsmList);
        begin
          inc(lists);
          list[lists]:=p;
        end;

      begin
        to_do:=[low(Tasmlisttype)..high(Tasmlisttype)];
        if usedeffileforexports then
          exclude(to_do,al_exports);
        if not(tf_section_threadvars in target_info.flags) then
          exclude(to_do,al_threadvars);
        for i:=low(TasmlistType) to high(TasmlistType) do
          if (i in to_do) and (current_asmdata.asmlists[i]<>nil) and
             (not current_asmdata.asmlists[i].empty) then
            addlist(current_asmdata.asmlists[i]);

        if SmartAsm then
          writetreesmart
        else
          writetree;
      end;


{*****************************************************************************
                     Generate Assembler Files Main Procedure
*****************************************************************************}

    Procedure GenerateAsm(smart:boolean);
      var
        a : TAssembler;
      begin
        if not assigned(CAssembler[target_asm.id]) then
          Message(asmw_f_assembler_output_not_supported);
        a:=CAssembler[target_asm.id].Create(@target_asm,smart);
        a.MakeObject;
        a.Free;
      end;


    function GetExternalGnuAssemblerWithAsmInfoWriter(info: pasminfo; wr: TExternalAssemblerOutputFile): TExternalAssembler;
      var
        asmkind: tasm;
      begin
        for asmkind in [as_gas,as_ggas,as_darwin] do
          if assigned(asminfos[asmkind]) and
             (target_info.system in asminfos[asmkind]^.supported_targets) then
            begin
              result:=TExternalAssemblerClass(CAssembler[asmkind]).CreateWithWriter(asminfos[asmkind],wr,false,false);
              exit;
            end;
        Internalerror(2015090604);
      end;

{*****************************************************************************
                                 Init/Done
*****************************************************************************}

    procedure RegisterAssembler(const r:tasminfo;c:TAssemblerClass);
      var
        t : tasm;
      begin
        t:=r.id;
        if assigned(asminfos[t]) then
          writeln('Warning: Assembler is already registered!')
        else
          Getmem(asminfos[t],sizeof(tasminfo));
        asminfos[t]^:=r;
        CAssembler[t]:=c;
      end;

end.
