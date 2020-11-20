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
      systems,globtype,globals,aasmbase,aasmtai,aasmdata,ogbase,owbase,finput;

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

      TExternalAssembler = class;

      IExternalAssemblerOutputFileDecorator=interface
        function LinePrefix: AnsiString;
        function LinePostfix: AnsiString;
        function LineFilter(const s: AnsiString): AnsiString;
        function LineEnding(const deflineending: ShortString): ShortString;
      end;

      TExternalAssemblerOutputFile=class
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
      {input source info}
        lastfileinfo : tfileposinfo;
        infile,
        lastinfile   : tinputfile;
      {last section type written}
        lastsectype : TAsmSectionType;
        procedure WriteSourceLine(hp: tailineinfo);
        procedure WriteTempalloc(hp: tai_tempalloc);
        procedure WriteRealConstAsBytes(hp: tai_realconst; const dbdir: string; do_line: boolean);
        function WriteComments(var hp: tai): boolean;
        function single2str(d : single) : string; virtual;
        function double2str(d : double) : string; virtual;
        function extended2str(e : extended) : string; virtual;
        Function DoPipe:boolean; virtual;

        function CreateNewAsmWriter: TExternalAssemblerOutputFile; virtual;

        {# Return true if the external assembler should run again }
        function RerunAssembler: boolean; virtual;
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
        Constructor Create(info: pasminfo; smart: boolean); override; final;
        Constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); virtual;
        procedure MakeObject;override;
        destructor Destroy; override;

        property writer: TExternalAssemblerOutputFile read fwriter;
      end;
      TExternalAssemblerClass = class of TExternalAssembler;

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
{$ifdef OMFOBJSUPPORT}
      omfbase,
      ogomf,
{$endif OMFOBJSUPPORT}
{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
{$else}
{$ifdef FPC_SOFT_FPUX80}
      sfpux80,
{$endif FPC_SOFT_FPUX80}
{$endif}
      cscript,fmodule,verbose,
      cpubase,cpuinfo,triplet,
      aasmcpu;

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
        s:='';
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
      var
        newline: pshortstring;
        newlineres: shortstring;
        index: longint;
      begin
        MaybeAddLinePostfix;
        if (cs_assemble_on_target in current_settings.globalswitches) then
          newline:=@target_info.newline
        else
          newline:=@source_info.newline;
        if assigned(decorator) then
          begin
            newlineres:=decorator.LineEnding(newline^);
            newline:=@newlineres;
          end;
        if OutCnt>=AsmOutSize-length(newline^) then
         AsmFlush;
        index:=1;
        repeat
          OutBuf[OutCnt]:=newline^[index];
          inc(OutCnt);
          inc(AsmSize);
          inc(index);
        until index>length(newline^);
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
        if (([cs_asm_extern,cs_asm_leave,cs_assemble_on_target] * current_settings.globalswitches) = []) then
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
{$ifdef hasunix}
        DoPipe:=(cs_asm_pipe in current_settings.globalswitches) and
                (([cs_asm_extern,cs_asm_leave,cs_assemble_on_target] * current_settings.globalswitches) = []) and
                ((asminfo^.id in [as_gas,as_ggas,as_darwin,as_powerpc_xcoff,as_clang_gas,as_clang_llvm,as_solaris_as]));
{$else hasunix}
        DoPipe:=false;
{$endif}
      end;


    function TExternalAssembler.CreateNewAsmWriter: TExternalAssemblerOutputFile;
      begin
        result:=TExternalAssemblerOutputFile.Create(self);
      end;


    Constructor TExternalAssembler.Create(info: pasminfo; smart: boolean);
      begin
        CreateWithWriter(info,CreateNewAsmWriter,true,smart);
      end;


    constructor TExternalAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter,smart: boolean);
      begin
        inherited Create(info,smart);
        fwriter:=wr;
        ffreewriter:=freewriter;
        if SmartAsm then
          begin
            path:=FixPath(ChangeFileExt(AsmFileName,target_info.smartext),false);
            CreateSmartLinkPath(path);
          end;
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
        asmbin : TCmdStr;
      begin
        asfound:=false;
        asmbin:=asminfo^.asmbin;
        if (af_llvm in asminfo^.flags) then
          asmbin:=asmbin+llvmutilssuffix;
        if cs_assemble_on_target in current_settings.globalswitches then
         begin
           { If assembling on target, don't add any path PM }
           FindAssembler:=utilsprefix+ChangeFileExt(asmbin,target_info.exeext);
           exit;
         end
        else
         UtilExe:=utilsprefix+ChangeFileExt(asmbin,source_info.exeext);
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
        result:=true;
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

        repeat
          result:=CallAssembler(FindAssembler,MakeCmdLine)
        until not(result) or not RerunAssembler;
        if result then
          writer.RemoveAsm
        else
          GenerateError;
      end;


    function TExternalAssembler.MakeCmdLine: TCmdStr;

      function section_high_bound:longint;
        var
          alt : tasmlisttype;
        begin
          result:=0;
          for alt:=low(tasmlisttype) to high(tasmlisttype) do
            result:=result+current_asmdata.asmlists[alt].section_count;
        end;

      const
        min_big_obj_section_count = $7fff;

      begin
        result:=asminfo^.asmcmd;
        if af_llvm in target_asm.flags then
          Replace(result,'$TRIPLET',targettriplet(triplet_llvm))
{$ifdef arm}
        else if (target_info.system=system_arm_ios) then
          Replace(result,'$ARCH',lower(cputypestr[current_settings.cputype]))
{$endif arm}
        ;
        if (cs_assemble_on_target in current_settings.globalswitches) then
         begin
           Replace(result,'$ASM',maybequoted(ScriptFixFileName(AsmFileName)));
           Replace(result,'$OBJ',maybequoted(ScriptFixFileName(ObjFileName)));
         end
        else
         begin
{$ifdef hasunix}
          if DoPipe then
            if not(asminfo^.id in [as_clang_gas,as_clang_asdarwin,as_clang_llvm]) then
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

         if target_info.endian=endian_little then
           Replace(result,'$ENDIAN','-mlittle')
         else
           Replace(result,'$ENDIAN','-mbig');

         { as we don't keep track of the amount of sections we created we simply
           enable Big Obj COFF files always for targets that need them }
         if (cs_asm_pre_binutils_2_25 in current_settings.globalswitches) or
            not (target_info.system in systems_all_windows+systems_nativent-[system_i8086_win16]) or
            (section_high_bound<min_big_obj_section_count) then
           Replace(result,'$BIGOBJ','')
         else
           Replace(result,'$BIGOBJ','-mbig-obj');

         Replace(result,'$EXTRAOPT',asmextraopt);
      end;


    function TExternalAssembler.RerunAssembler: boolean;
      begin
        result:=false;
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
{$else}
{$ifdef FPC_SOFT_FPUX80}
	eextended: floatx80;
{$endif}
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
{$else}
{$ifdef FPC_SOFT_FPUX80}
{$push}{$warn 6018 off} { Unreachable code due to compile time evaluation }
             aitrealconst_s80bit:
               begin
     	         if sizeof(tai_realconst(hp).value.s80val) = sizeof(double) then
                   writer.AsmWriteLn(asminfo^.comment+'value: '+double2str(tai_realconst(hp).value.s80val))
     	         else if sizeof(tai_realconst(hp).value.s80val) = sizeof(single) then
                   writer.AsmWriteLn(asminfo^.comment+'value: '+single2str(tai_realconst(hp).value.s80val))
                else
     	         internalerror(2017091901);
       	      end;
{$pop}
{$endif}
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
{$else}
{$ifdef FPC_SOFT_FPUX80}
{$push}{$warn 6018 off} { Unreachable code due to compile time evaluation }
          aitrealconst_s80bit:
            begin
	      if sizeof(tai_realconst(hp).value.s80val) = sizeof(double) then
                eextended:=float64_to_floatx80(float64(double(tai_realconst(hp).value.s80val)))
	      else if sizeof(tai_realconst(hp).value.s80val) = sizeof(single) then
	        eextended:=float32_to_floatx80(float32(single(tai_realconst(hp).value.s80val)))
	      else
	        internalerror(2017091902);
              pdata:=@eextended;
            end;
{$pop}
{$endif}
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


    function TExternalAssembler.WriteComments(var hp: tai): boolean;
      begin
        result:=true;
        case hp.typ of
          ait_comment :
            Begin
              writer.AsmWrite(asminfo^.comment);
              writer.AsmWritePChar(tai_comment(hp).str);
              writer.AsmLn;
            End;

          ait_regalloc :
            begin
              if (cs_asm_regalloc in current_settings.globalswitches) then
                begin
                  writer.AsmWrite(#9+asminfo^.comment+'Register ');
                  repeat
                    writer.AsmWrite(std_regname(Tai_regalloc(hp).reg));
                    if (hp.next=nil) or
                       (tai(hp.next).typ<>ait_regalloc) or
                       (tai_regalloc(hp.next).ratype<>tai_regalloc(hp).ratype) then
                      break;
                    hp:=tai(hp.next);
                    writer.AsmWrite(',');
                  until false;
                  writer.AsmWrite(' ');
                  writer.AsmWriteLn(regallocstr[tai_regalloc(hp).ratype]);
                end;
            end;

          ait_tempalloc :
            begin
              if (cs_asm_tempalloc in current_settings.globalswitches) then
                WriteTempalloc(tai_tempalloc(hp));
            end;

          ait_varloc:
            begin
              { ait_varloc is present here only when register allocation is not done ( -sr option ) }
              if tai_varloc(hp).newlocationhi<>NR_NO then
                writer.AsmWriteLn(asminfo^.comment+'Var '+tai_varloc(hp).varsym.realname+' located in register '+
                  std_regname(tai_varloc(hp).newlocationhi)+':'+std_regname(tai_varloc(hp).newlocation))
              else
                writer.AsmWriteLn(asminfo^.comment+'Var '+tai_varloc(hp).varsym.realname+' located in register '+
                  std_regname(tai_varloc(hp).newlocation));
            end;
          else
            result:=false;
        end;
      end;


    procedure TExternalAssembler.WriteTree(p:TAsmList);
      begin
      end;


    procedure TExternalAssembler.WriteAsmList;
      begin
      end;


    procedure TExternalAssembler.MakeObject;
      begin
        writer.AsmCreate(cut_normal);
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
            ObjData.Writereloc(stab.nvalue,4,relocsym,RELOC_ABSOLUTE32);
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
            indsym:=ObjData.CObjSymbol.Create(ObjData.ObjSymbolList, indirectname);
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
        cpu: tcputype;
        eabi_section, TmpSection: TObjSection;
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
                     { may need to increase alignment of section }
                     if tai_align_abstract(hp).aligntype>ObjData.CurrObjSec.secalign then
                       ObjData.CurrObjSec.secalign:=tai_align_abstract(hp).aligntype;
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
             ait_realconst:
               ObjData.alloc(tai_realconst(hp).savesize);
             ait_const:
               begin
                 { if symbols are provided we can calculate the value for relative symbols.
                   This is required for length calculation of leb128 constants }
                 if assigned(tai_const(hp).sym) then
                   begin
                     objsym:=Objdata.SymbolRef(tai_const(hp).sym);
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
{$push} {$R-}{$Q-}
                             else
                               Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
                           end;
{$pop}
                       end;
                   end;
                 ObjData.alloc(tai_const(hp).size);
               end;
             ait_directive:
               begin
                 case tai_directive(hp).directive of
                   asd_indirect_symbol:
                     { handled in TreePass1 }
                     ;
                   asd_lazy_reference:
                     begin
                       if tai_directive(hp).name='' then
                         Internalerror(2009112101);
                       objsym:=ObjData.symbolref(tai_directive(hp).name);
                       objsym.bind:=AB_LAZY;
                     end;
                   asd_reference:
                     { ignore for now, but should be added}
                     ;
                   asd_cpu:
                     begin
                       ObjData.CPUType:=cpu_none;
                       for cpu:=low(tcputype) to high(tcputype) do
                         if cputypestr[cpu]=tai_directive(hp).name then
                           begin
                             ObjData.CPUType:=cpu;
                             break;
                           end;
                     end;
{$ifdef OMFOBJSUPPORT}
                   asd_omf_linnum_line:
                     { ignore for now, but should be added}
                     ;
{$endif OMFOBJSUPPORT}
{$ifdef ARM}
                   asd_thumb_func:
                     ObjData.ThumbFunc:=true;
                   asd_code:
                     { ai_directive(hp).name can be only 16 or 32, this is checked by the reader }
                     ObjData.ThumbFunc:=tai_directive(hp).name='16';
{$endif ARM}
{$ifdef RISCV}
                   asd_option:
                     internalerror(2019031701);
{$endif RISCV}
                   else
                     internalerror(2010011101);
                 end;
               end;
             ait_section:
               begin
                 if Tai_section(hp).sectype=sec_user then
                   ObjData.CreateSection(Tai_section(hp).sectype,Tai_section(hp).secflags,Tai_section(hp).secprogbits,Tai_section(hp).name^,Tai_section(hp).secorder)
                 else
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
             ait_eabi_attribute :
               begin
                 eabi_section:=ObjData.findsection('.ARM.attributes');
                 if not(assigned(eabi_section)) then
                   begin
                     TmpSection:=ObjData.CurrObjSec;
                     ObjData.CreateSection(sec_arm_attribute,[],SPB_ARM_ATTRIBUTES,'',secorder_default);
                     eabi_section:=ObjData.CurrObjSec;
                     ObjData.setsection(TmpSection);
                   end;
                 if eabi_section.Size=0 then
                   eabi_section.alloc(16);
                 eabi_section.alloc(LengthUleb128(tai_eabi_attribute(hp).tag));
                 case tai_eabi_attribute(hp).eattr_typ of
                   eattrtype_dword:
                     eabi_section.alloc(LengthUleb128(tai_eabi_attribute(hp).value));
                   eattrtype_ntbs:
                     eabi_section.alloc(Length(tai_eabi_attribute(hp).valuestr^)+1);
                   else
                     Internalerror(2019100701);
                 end;
               end;
             else
               ;
           end;
           hp:=Tai(hp.next);
         end;
        TreePass0:=hp;
      end;


    function TInternalAssembler.TreePass1(hp:Tai):Tai;
      var
        objsym,
        objsymend : TObjSymbol;
        cpu: tcputype;
        eabi_section: TObjSection;
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

                     { maximum number of bytes for alignment exeeded? }
                     if (Tai_align_abstract(hp).aligntype<>Tai_align_abstract(hp).maxbytes) and
                       (Tai_align_abstract(hp).fillsize>Tai_align_abstract(hp).maxbytes) then
                       Tai_align_abstract(hp).fillsize:=align(ObjData.CurrObjSec.Size,Byte(Tai_align_abstract(hp).aligntype div 2))-
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
             ait_realconst:
               ObjData.alloc(tai_realconst(hp).savesize);
             ait_const:
               begin
                 { Recalculate relative symbols }
                 if assigned(tai_const(hp).sym) and
                    assigned(tai_const(hp).endsym) then
                   begin
                     objsym:=Objdata.SymbolRef(tai_const(hp).sym);
                     objsymend:=Objdata.SymbolRef(tai_const(hp).endsym);
                     if Tai_const(hp).consttype in [aitconst_gottpoff,aitconst_tlsgd,aitconst_tlsdesc] then
                       begin
                         if objsymend.objsection<>ObjData.CurrObjSec then
                           Internalerror(2019092801);
                         Tai_const(hp).value:=objsymend.address-ObjData.CurrObjSec.Size+Tai_const(hp).symofs;
                       end
                     else if objsymend.objsection<>objsym.objsection then
                       begin
                         if (Tai_const(hp).consttype in [aitconst_uleb128bit,aitconst_sleb128bit]) or
                            (objsym.objsection<>ObjData.CurrObjSec) then
                           internalerror(200905042);
                       end
{$push} {$R-}{$Q-}
                     else
                       Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
                   end;
{$pop}
                 if (Tai_const(hp).consttype in [aitconst_uleb128bit,aitconst_sleb128bit]) then
                   Tai_const(hp).fixsize;
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
             ait_directive :
               begin
                 case tai_directive(hp).directive of
                   asd_indirect_symbol:
                     if tai_directive(hp).name='' then
                       Internalerror(2009101103)
                     else if not SetIndirectToSymbol(Tai(hp.Previous), tai_directive(hp).name) then
                       Internalerror(2009101102);
                   asd_lazy_reference:
                     { handled in TreePass0 }
                     ;
                   asd_reference:
                     { ignore for now, but should be added}
                     ;
                   asd_thumb_func:
                     { ignore for now, but should be added}
                     ;
                   asd_code:
                     { ignore for now, but should be added}
                     ;
                   asd_option:
                     { ignore for now, but should be added}
                     ;
{$ifdef OMFOBJSUPPORT}
                   asd_omf_linnum_line:
                     { ignore for now, but should be added}
                     ;
{$endif OMFOBJSUPPORT}
                   asd_cpu:
                     begin
                       ObjData.CPUType:=cpu_none;
                       for cpu:=low(tcputype) to high(tcputype) do
                         if cputypestr[cpu]=tai_directive(hp).name then
                           begin
                             ObjData.CPUType:=cpu;
                             break;
                           end;
                     end;
                   else
                     internalerror(2010011102);
                 end;
               end;
             ait_eabi_attribute :
               begin
                 eabi_section:=ObjData.findsection('.ARM.attributes');
                 if not(assigned(eabi_section)) then
                   Internalerror(2019100702);
                 if eabi_section.Size=0 then
                   eabi_section.alloc(16);
                 eabi_section.alloc(LengthUleb128(tai_eabi_attribute(hp).tag));
                 case tai_eabi_attribute(hp).eattr_typ of
                   eattrtype_dword:
                     eabi_section.alloc(LengthUleb128(tai_eabi_attribute(hp).value));
                   eattrtype_ntbs:
                     eabi_section.alloc(Length(tai_eabi_attribute(hp).valuestr^)+1);
                   else
                     Internalerror(2019100703);
                 end;
               end;
             else
               ;
           end;
           hp:=Tai(hp.next);
         end;
        TreePass1:=hp;
      end;


    function TInternalAssembler.TreePass2(hp:Tai):Tai;
      var
        fillbuffer : tfillbuffer;
        leblen : byte;
        lebbuf : array[0..63] of byte;
        objsym,
        ref,
        objsymend : TObjSymbol;
        zerobuf : array[0..63] of byte;
        relative_reloc: boolean;
        pdata : pointer;
        ssingle : single;
        ddouble : double;
        {$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
        eextended : extended;
	{$else}
        {$ifdef FPC_SOFT_FPUX80}
	eextended : floatx80;
        {$endif}
        {$endif}
        ccomp : comp;
        tmp    : word;
        cpu: tcputype;
        ddword : dword;
        eabi_section: TObjSection;
        s: String;
        TmpDataPos: TObjSectionOfs;
      begin
        fillchar(zerobuf,sizeof(zerobuf),0);
        fillchar(objsym,sizeof(objsym),0);
        fillchar(objsymend,sizeof(objsymend),0);
        { main loop }
        while assigned(hp) do
         begin
           case hp.typ of
             ait_align :
               begin
                 if tai_align_abstract(hp).aligntype>ObjData.CurrObjSec.secalign then
                   InternalError(2012072301);
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
             ait_realconst:
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
         {$else}
         {$ifdef FPC_SOFT_FPUX80}
           {$push}{$warn 6018 off} { Unreachable code due to compile time evaluation }
                   aitrealconst_s80bit:
                     begin
		       if sizeof(tai_realconst(hp).value.s80val) = sizeof(double) then
                         eextended:=float64_to_floatx80(float64(double(tai_realconst(hp).value.s80val)))
		       else if sizeof(tai_realconst(hp).value.s80val) = sizeof(single) then
			 eextended:=float32_to_floatx80(float32(single(tai_realconst(hp).value.s80val)))
		       else
			 internalerror(2017091903);
                       pdata:=@eextended;
                     end;
           {$pop}
	 {$endif}
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
                 relative_reloc:=false;
                 if assigned(tai_const(hp).sym) and
                    assigned(tai_const(hp).endsym) then
                   begin
                     objsym:=Objdata.SymbolRef(tai_const(hp).sym);
                     objsymend:=Objdata.SymbolRef(tai_const(hp).endsym);
                     relative_reloc:=(objsym.objsection<>objsymend.objsection);
                     if Tai_const(hp).consttype in [aitconst_gottpoff] then
                       begin
                         if objsymend.objsection<>ObjData.CurrObjSec then
                           Internalerror(2019092802);
                         Tai_const(hp).value:=objsymend.address-ObjData.CurrObjSec.Size+Tai_const(hp).symofs;
                       end
                     else if Tai_const(hp).consttype in [aitconst_tlsgd,aitconst_tlsdesc] then
                       begin
                         if objsymend.objsection<>ObjData.CurrObjSec then
                           Internalerror(2019092803);
                         Tai_const(hp).value:=ObjData.CurrObjSec.Size-objsymend.address+Tai_const(hp).symofs;
                       end
                     else if objsymend.objsection<>objsym.objsection then
                       begin
                         if (Tai_const(hp).consttype in [aitconst_uleb128bit,aitconst_sleb128bit]) or
                            (objsym.objsection<>ObjData.CurrObjSec) then
                           internalerror(2019010301);
                       end
                     else
{$push} {$R-}{$Q-}
                       Tai_const(hp).value:=objsymend.address-objsym.address+Tai_const(hp).symofs;
                   end;
{$pop}
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
                       else if relative_reloc then
                         ObjData.writereloc(ObjData.CurrObjSec.size+tai_const(hp).size-objsym.address+tai_const(hp).symofs,tai_const(hp).size,objsymend,RELOC_RELATIVE)
                       else
                         ObjData.writebytes(Tai_const(hp).value,tai_const(hp).size);
                     end;
                   aitconst_rva_symbol :
                     begin
                       { PE32+? }
                       if target_info.system in systems_peoptplus then
                         ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_RVA)
                       else
                         ObjData.writereloc(Tai_const(hp).symofs,sizeof(pint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_RVA);
                     end;
                   aitconst_secrel32_symbol :
                     begin
                       { Required for DWARF2 support under Windows }
                       ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_SECREL32);
                     end;
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
{                   aitconst_gottpoff:
                     ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_TPOFF); }
                   aitconst_tpoff:
                     ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_TPOFF);
                   aitconst_tlsgd:
                     ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_TLSGD);
                   aitconst_tlsdesc:
                     begin
                       { must be a relative symbol, thus value being valid }
                       if not(assigned(tai_const(hp).sym)) or not(assigned(tai_const(hp).endsym)) then
                         Internalerror(2019092904);
                       ObjData.writereloc(Tai_const(hp).value,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_TLSDESC);
                     end;
{$endif arm}
                   aitconst_dtpoff:
                     { so far, the size of dtpoff is fixed to 4 bytes }
                     ObjData.writereloc(Tai_const(hp).symofs,4,Objdata.SymbolRef(tai_const(hp).sym),RELOC_DTPOFF);
                   aitconst_gotoff_symbol:
                     ObjData.writereloc(Tai_const(hp).symofs,sizeof(longint),Objdata.SymbolRef(tai_const(hp).sym),RELOC_GOTOFF);
                   aitconst_uleb128bit,
                   aitconst_sleb128bit :
                     begin
                       if Tai_const(hp).fixed_size=0 then
                         Internalerror(2019030302);
                       if tai_const(hp).consttype=aitconst_uleb128bit then
                         leblen:=EncodeUleb128(qword(Tai_const(hp).value),lebbuf,Tai_const(hp).fixed_size)
                       else
                         leblen:=EncodeSleb128(Tai_const(hp).value,lebbuf,Tai_const(hp).fixed_size);
                       if leblen<>tai_const(hp).fixed_size then
                         internalerror(200709271);
                       ObjData.writebytes(lebbuf,leblen);
                     end;
                   aitconst_darwin_dwarf_delta32,
                   aitconst_darwin_dwarf_delta64:
                     ObjData.writebytes(Tai_const(hp).value,tai_const(hp).size);
                   aitconst_half16bit,
                   aitconst_gs:
                     begin
                       tmp:=Tai_const(hp).value div 2;
                       ObjData.writebytes(tmp,2);
                     end;
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
             ait_directive :
               begin
                 case tai_directive(hp).directive of
                   asd_weak_definition,
                   asd_weak_reference:
                     begin
                       objsym:=ObjData.symbolref(tai_directive(hp).name);
                       if objsym.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL] then
                         objsym.bind:=AB_WEAK_EXTERNAL
                       else
                         { TODO: should become a weak definition; for now, do
                             the same as what was done for ait_weak }
                         objsym.bind:=AB_WEAK_EXTERNAL;
                     end;
                   asd_cpu:
                     begin
                       ObjData.CPUType:=cpu_none;
                       for cpu:=low(tcputype) to high(tcputype) do
                         if cputypestr[cpu]=tai_directive(hp).name then
                           begin
                             ObjData.CPUType:=cpu;
                             break;
                           end;
                     end;
{$ifdef OMFOBJSUPPORT}
                   asd_omf_linnum_line:
                     begin
                       TOmfObjSection(ObjData.CurrObjSec).LinNumEntries.Add(
                         TOmfSubRecord_LINNUM_MsLink_Entry.Create(
                           strtoint(tai_directive(hp).name),
                           ObjData.CurrObjSec.Size
                         ));
                     end;
{$endif OMFOBJSUPPORT}
                   else
                     ;
                 end
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
             ait_eabi_attribute :
               begin
                 eabi_section:=ObjData.findsection('.ARM.attributes');
                 if not(assigned(eabi_section)) then
                   Internalerror(2019100704);
                 if eabi_section.Size=0 then
                   begin
                     s:='A';
                     eabi_section.write(s[1],1);
                     ddword:=eabi_section.Size-1;
                     eabi_section.write(ddword,4);
                     s:='aeabi'#0;
                     eabi_section.write(s[1],6);
                     s:=#1;
                     eabi_section.write(s[1],1);
                     ddword:=eabi_section.Size-1-4-6-1;
                     eabi_section.write(ddword,4);
                   end;
                 leblen:=EncodeUleb128(tai_eabi_attribute(hp).tag,lebbuf,0);
                 eabi_section.write(lebbuf,leblen);

                 case tai_eabi_attribute(hp).eattr_typ of
                   eattrtype_dword:
                     begin
                       leblen:=EncodeUleb128(tai_eabi_attribute(hp).value,lebbuf,0);
                       eabi_section.write(lebbuf,leblen);
                     end;
                   eattrtype_ntbs:
                     begin
                       s:=tai_eabi_attribute(hp).valuestr^+#0;
                       eabi_section.write(s[1],Length(s));
                     end
                   else
                     Internalerror(2019100705);
                 end;
                 { update size of attributes section, write directly to the dyn. arrays as
                   we do not increase the size of section }
                 TmpDataPos:=eabi_section.Data.Pos;
                 eabi_section.Data.seek(1);
                 ddword:=eabi_section.Size-1;
                 eabi_section.Data.write(ddword,4);
                 eabi_section.Data.seek(12);
                 ddword:=eabi_section.Size-1-4-6;
                 eabi_section.Data.write(ddword,4);
                 eabi_section.Data.Seek(TmpDataPos);
               end;
             else
               ;
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
        { leave if errors have occurred }
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

        { leave if errors have occurred }
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

        { don't write the .o file if errors have occurred }
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
           { leave if errors have occurred }
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

           { leave if errors have occurred }
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

           { leave if errors have occurred }
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
        for asmkind in [as_gas,as_ggas,as_darwin,as_clang_gas,as_clang_asdarwin] do
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
