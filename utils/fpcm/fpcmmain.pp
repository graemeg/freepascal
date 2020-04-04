{
    Copyright (c) 2001 by Peter Vreman

    FPCMake - Main module

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$ifdef fpc}{$mode objfpc}{$endif}
{$H+}
unit fpcmmain;
interface

    uses
      dos,
{$ifdef Unix}
  {$ifdef VER1_0}
    {$ifdef linux}
     {$ifndef BSD}
      linux,
     {$endif}
    {$endif}
    {$ifdef BSD}
      Linux,
    {$endif}
  {$else}
     baseunix,
     unix,
  {$endif}
{$endif}
      sysutils,classes,
      fpcmdic;

{$ifdef BEOS}
{$define NO_UNIX_UNIT}
{$endif}
{$ifdef SOLARIS}
{$define NO_UNIX_UNIT}
{$endif}
{$ifdef QNX}
{$define NO_UNIX_UNIT}
{$endif}

{$ifdef OS2}
  {$define LIMIT83}
{$endif}
{$ifdef EMX}
  {$define LIMIT83}
{$endif}
{$ifdef GO32V2}
  {$define LIMIT83}
{$endif}

    const
      Version='2.0.0';
      Title='FPCMake Version '+Version;
{$ifdef REVINC}
      DateRevision={$i revision.inc};
{$else}
      DateRevision={$I %DATE%}+' no revision';
{$endif}
      TitleNoDate=Title;
      TitleDate=Title+' '+DateRevision;

    type
      TCpu=(
        c_i386,c_m68k,c_powerpc,c_sparc,c_x86_64,c_arm,c_powerpc64,c_avr,c_armeb,c_armel,c_mips,c_mipsel,c_mips64,c_mips64el,c_jvm,c_i8086,c_aarch64,c_wasm,c_sparc64,c_riscv32,c_riscv64,c_xtensa
      );

      TOS=(
        o_linux,o_go32v2,o_win32,o_os2,o_freebsd,o_beos,o_haiku,o_netbsd,
        o_amiga,o_atari, o_solaris, o_qnx, o_netware, o_openbsd,o_wdosx,
        o_palmos,o_macos,o_darwin,o_emx,o_watcom,o_morphos,o_netwlibc,
        o_win64,o_wince,o_gba,o_nds,o_embedded,o_symbian,o_nativent,o_iphonesim,
        o_wii,o_aix,o_java,o_android,o_msdos,o_aros,o_dragonfly,o_win16,o_wasm,o_freertos
      );

      TTargetSet=array[tcpu,tos] of boolean;

    const
      CpuStr : array[TCpu] of string=(
        'i386','m68k','powerpc','sparc','x86_64','arm','powerpc64','avr','armeb', 'armel', 'mips', 'mipsel', 'mips64', 'mips64el', 'jvm','i8086','aarch64','wasm','sparc64','riscv32','riscv64','xtensa'
      );

      CpuSuffix : array[TCpu] of string=(
        '_i386','_m68k','_powerpc','_sparc','_x86_64','_arm','_powerpc64','_avr','_armeb', '_armel', '_mips', '_mipsel', '_mips64', '_mips64el', '_jvm','_i8086','_aarch64','_wasm','_sparc64','_riscv32','_riscv64','xtensa'
      );

      ppcSuffix : array[TCpu] of string=(
        '386','68k','ppc','sparc','x64','arm','ppc64','avr','armeb', 'armel', 'mips', 'mipsel', 'mips64', 'mips64el', 'jvm','8086','a64','wasm','sparc64','rv32','rv64','xtensa'
      );

      OSStr : array[TOS] of string=(
        'linux','go32v2','win32','os2','freebsd','beos','haiku','netbsd',
        'amiga','atari','solaris', 'qnx', 'netware','openbsd','wdosx',
        'palmos','macos','darwin','emx','watcom','morphos','netwlibc',
        'win64','wince','gba','nds','embedded','symbian','nativent',
        'iphonesim', 'wii', 'aix', 'java', 'android', 'msdos', 'aros',
        'dragonfly', 'win16', 'wasm', 'freertos'
      );

      OSSuffix : array[TOS] of string=(
        '_linux','_go32v2','_win32','_os2','_freebsd','_beos','_haiku','_netbsd',
        '_amiga','_atari','_solaris', '_qnx', '_netware','_openbsd','_wdosx',
        '_palmos','_macos','_darwin','_emx','_watcom','_morphos','_netwlibc',
        '_win64','_wince','_gba','_nds','_embedded','_symbian','_nativent',
        '_iphonesim','_wii','_aix','_java','_android','_msdos','_aros',
        '_dragonfly','_win16','_wasm','_freertos'
      );

      { This table is kept OS,Cpu because it is easier to maintain (PFV) }
      OSCpuPossible : array[TOS,TCpu] of boolean = (
        { os          i386    m68k  ppc    sparc  x86_64 arm    ppc64  avr    armeb  armel  mips   mipsel mips64 misp64el jvm    i8086  aarch64 wasm sparc64 riscv32 riscv64 xtensa}
        { linux }   ( true,  true,  true,  true,  true,  true,  true,  false, true,  false, true,  true,  false, false,   false, false, true,  false, true,  true,   true,    true ),
        { go32v2 }  ( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { win32 }   ( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { os2 }     ( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { freebsd } ( true,  false, false, false, true,  false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { beos }    ( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { haiku }   ( true,  false, false, false, true,  false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { netbsd }  ( true,  true,  true,  true,  true,  true,  false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { amiga }   ( false, true,  true,  false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { atari }   ( false, true,  false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { solaris } ( true,  false, false, true,  true,  false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { qnx }     ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { netware } ( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { openbsd } ( true,  false, false, false, true,  false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { wdosx }   ( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { palmos }  ( false, true,  false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { macos }   ( false, true,  true,  false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { darwin }  ( true,  false, true,  false, true,  true,  true,  false, false, false, false, false, false, false,   false, false, true,  false, false, false,  false,   false),
        { emx }     ( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { watcom }  ( true,  false, false, false ,false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { morphos } ( false, false, true,  false ,false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { netwlibc }( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { win64   } ( false, false, false, false, true,  false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { wince    }( true,  false, false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { gba    }  ( false, false, false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { nds    }  ( false, false, false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { embedded }( true,  true,  true,  true,  true,  true,  true,  true,  true , false, false, true , false, false,   false, true , false, false, false, true,   true,    true ),
        { symbian } ( true,  false, false, false, false, true,  false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { nativent }( true,  false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { iphonesim }( true, false, false, false, true,  false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { wii }     ( false, false, true,  false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { aix }     ( false, false, true,  false, false, false, true,  false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { java }    ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   true,  false, false, false, false, false,  false,   false),
        { android } ( true,  false, false, false, true,  true,  false, false, false, false, false, true,  false, false,   true,  false, true,  false, false, false,  false,   false),
        { msdos }   ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, true , false, false, false, false,  false,   false),
        { aros }    ( true,  false, false, false, true,  true,  false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        {dragonfly} ( false, false, false, false, true,  false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   false),
        { win16 }   ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, true , false, false, false, false,  false,   false),
        { wasm }    ( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, true,  false, false,  false,   false),
        { freertos }( false, false, false, false, false, false, false, false, false, false, false, false, false, false,   false, false, false, false, false, false,  false,   true )
      );

    type
      TKeyValueItem = class(TDictionaryItem)
      private
        FValue : string;
      public
        constructor Create(const k,v:string);
        property Value:string read FValue write FValue;
      end;

      TKeyValue = class(TDictionary)
      private
        function GetKey(const k:string):string;
      public
        procedure Add(const k,v:String);
        property Key[const s:string]:string read GetKey write Add;default;
      end;

      TFPCMakeSection = class(TDictionaryItem)
      private
        FList       : TStringList;
        FDictionary : TKeyValue;
        procedure BuildIniDic(p:TDictionaryItem);
        procedure BuildMakefileDic(p:TDictionaryItem);
        function GetKey(const k:string):string;
      public
        constructor Create(const n:string);
        constructor CreateKeyValue(const n:string);
        destructor  Destroy;override;
        procedure AddLine(const s:string);
        procedure AddKey(const k,v:string);
        procedure Clear;
        procedure ParseIni;
        procedure BuildIni;
        procedure BuildMakefile;
        property Key[const s:string]:string read GetKey;default;
        property List:TStringList read FList;
        property Dictionary:TKeyValue read FDictionary;
      end;

      TTargetRequireList = array[tcpu,tos] of TStringList;

      TFPCMakeVerbose = (FPCMakeError, FPCMakeInfo, FPCMakeDebug);

      TFPCMake = class
      private
        FStream         : TStream;
        FFileName       : string;
        FCommentChars   : TSysCharSet;
        FEmptyLines     : boolean;
        FSections       : TDictionary;
        FPackageSec,
        FExportSec      : TFPCMakeSection;
        FUsesLCL,
        FIsPackage      : boolean;
        FPackageName,
        FPackageVersion : string;
        FRequireList    : TTargetRequireList;
        FVariables      : TKeyValue;
        FIncludeTargets : TTargetSet;
        procedure Init;
        procedure ParseSec(p:TDictionaryItem);
        procedure PrintSec(p:TDictionaryItem);
        procedure PrintDic(p:TDictionaryItem);
        function  GetSec(const AName:string):TDictionaryItem;
        procedure LoadRequiredPackage(c:TCpu;t:TOS;const ReqName,ReqVersion:string);
        procedure LoadRequiredDir(c:TCpu;t:TOS;const MainPack,currdir,subdir:string);
        procedure LoadRequires(c:TCpu;t:TOS;FromFPCMake:TFPCMake);
        function  CopySection(Sec:TFPCMakeSection;Secname:string):TFPCMakeSection;
      protected
        VerboseIdent : string;
      public
        constructor Create(const AFileName:string);
        constructor CreateFromStream(s:TStream;const AFileName:string);
        destructor  Destroy;override;
        procedure Verbose(lvl:TFPCMakeVerbose;const s:string);virtual;
        procedure SetTargets(const s:string);
        procedure LoadSections;
        procedure LoadMakefileFPC;
        procedure LoadPackageSection;
        procedure LoadRequireSection;
        function  GetTargetRequires(c:TCpu;t:TOS):TStringList;
        function  CheckLibcRequire:boolean;
        procedure CreateExportSection;
        procedure AddFPCDefaultVariables;
        procedure AddLCLDefaultVariables;
        function  SubstVariables(const s:string):string;
        function  GetVariable(const inivar:string;dosubst:boolean):string;
        function  GetTargetVariable(c:TCPU;t:TOS;const inivar:string;dosubst:boolean):string;
        function  HasVariable(const inivar:string):boolean;
        function  HasTargetVariable(const inivar:string):boolean;
        function  SetVariable(const inivar,value:string;add:boolean):string;
        procedure Print;
        property Section[const s:string]:TDictionaryItem read GetSec;default;
        property RequireList:TTargetRequireList read FRequireList;
        property Variables:TKeyValue read FVariables;
        property UsesLCL:boolean read FUsesLCL;
        property IsPackage:boolean read FIsPackage;
        property PackageName:string read FPackageName;
        property PackageVersion:string read FPackageVersion;
        property PackageSec:TFPCMakeSection read FPackageSec;
        property ExportSec:TFPCMakeSection read FExportSec;
        property CommentChars:TSysCharSet read FCommentChars write FCommentChars;
        property EmptyLines:Boolean read FEmptyLines write FEmptyLines;
        property IncludeTargets:TTargetSet read FIncludeTargets write FIncludeTargets;
      end;

    function posidx(const substr,s : string;idx:integer):integer;
    function GetToken(var s:string;sep:char):string;
    procedure AddToken(var s:string;const tok:string;sep:char);
    procedure AddTokenNoDup(var s:string;const s2:string;sep:char);


implementation

    resourcestring
      s_not_list_sec='Not a list section "%s"';
      s_not_key_value_sec='Not a key-value section "%s"';
      s_err_section_start='%s:%d: Wrong section start';
      s_err_not_key_value='Parse error key=value excepted: "%s"';
      s_err_no_section='%s:%d: Entries without section';
      s_no_package_name='No package name set';
      s_no_package_version='No package version set';
      s_err_require_format='Wrong require format "%s"';
      s_wrong_package_name='Package name "%s" expected, but "%s" found';
      s_wrong_package_version='Package version "%s" expected, but version "%s" found';
      s_directory_not_found='Directory "%s" not found';
      s_makefilefpc_not_found='No Makefile.fpc found in directory "%s"';
      s_package_not_found='Target "%s", package "%s" not found';
      s_fpcmake_version_required='FPCMake version "%s" is required';
      s_no_targets_set='No targets set';
      s_targets_info='Targets: "%s"';
      s_globals='Globals:';


{****************************************************************************
                                 Helpers
****************************************************************************}

    Function PathExists ( F : String) : Boolean;
      Var
        Info : TSearchRec;
      begin
        if F[Length(f)] in ['/','\'] then
         Delete(f,length(f),1);
        PathExists:=(findfirst(F,faAnyFile,info)=0) and
                    ((info.attr and fadirectory)=fadirectory);
        findclose(Info);
      end;


    Function PathOrFileExists ( F : String) : Boolean;
      Var
        Info : Dos.SearchRec;
      begin
        if F[Length(f)] in ['/','\'] then
         Delete(f,length(f),1);
        dos.findfirst(f,fareadonly+faarchive+fahidden+fadirectory,info);
        PathOrFileExists:=(Doserror=0);
        dos.findclose(Info);
      end;


    function posidx(const substr,s : string;idx:integer):integer;
      var
        i,j : integer;
        e   : boolean;
      begin
        i:=idx;
        j:=0;
        e:=(length(SubStr)>0);
        while e and (i<=Length(s)-Length(SubStr)) do
         begin
           inc(i);
           if (SubStr[1]=s[i]) and (Substr=Copy(s,i,Length(SubStr))) then
            begin
              j:=i;
              e:=false;
            end;
         end;
        PosIdx:=j;
      end;


    function GetToken(var s:string;sep:char):string;
      var
        i : integer;
      begin
        s:=Trim(s);
        i:=pos(sep,s);
        if i=0 then
         begin
           Result:=s;
           s:='';
         end
        else
         begin
           Result:=Copy(s,1,i-1);
           Delete(s,1,i);
         end;
      end;


    procedure AddToken(var s:string;const tok:string;sep:char);
      begin
        if tok='' then
         exit;
        if s<>'' then
         s:=s+sep+tok
        else
         s:=tok;
      end;


    procedure AddTokenNoDup(var s:string;const s2:string;sep:char);
      var
        i,idx : integer;
        again,add : boolean;
      begin
        add:=false;
        idx:=0;
        repeat
          again:=false;
          i:=posidx(s2,s,idx);
          if (i=0) then
           add:=true
          else
           if (i=1) then
            begin
              if (length(s)>length(s2)) and
                 (s[length(s2)+1]<>sep) then
               add:=true;
            end
          else
           if (i>1) and
              ((s[i-1]<>sep) or
               ((length(s)>=i+length(s2)) and (s[i+length(s2)]<>sep))) then
            begin
              idx:=i+length(s2);
              again:=true;
            end;
        until not again;
        if add then
         begin
           if s='' then
            s:=s2
           else
            s:=s+sep+s2;
         end;
      end;



{****************************************************************************
                               TKeyValueItem
****************************************************************************}

    constructor TKeyValueItem.Create(const k,v:string);
      begin
        inherited Create(k);
        value:=v;
      end;


{****************************************************************************
                                 TKeyValue
****************************************************************************}

    function TKeyValue.GetKey(const k:string):string;
      var
        p : TKeyValueItem;
      begin
        p:=TKeyValueItem(Search(k));
        if p=nil then
         GetKey:=''
        else
         GetKey:=p.Value;
      end;


    procedure TKeyValue.Add(const k,v:string);
      var
        p : TKeyValueItem;
      begin
        p:=TKeyValueItem(Search(k));
        if p=nil then
         begin
           p:=TKeyValueItem.Create(k,v);
           Insert(p);
         end
        else
         p.Value:=v;
      end;


{****************************************************************************
                               TFPCMakeSection
****************************************************************************}

    constructor TFPCMakeSection.Create(const n:string);
      begin
        inherited Create(n);
        FList:=TStringList.Create;
        FDictionary:=nil;
      end;


    constructor TFPCMakeSection.CreateKeyValue(const n:string);
      begin
        inherited Create(n);
        FList:=nil;
        FDictionary:=TKeyValue.Create;
      end;


    destructor TFPCMakeSection.Destroy;
      begin
        inherited Destroy;
        FList.Free;
        FDictionary.Free;
      end;


    procedure TFPCMakeSection.Clear;
      begin
        FList.Free;
        FList:=TStringList.Create;
        FDictionary.Free;
        FDictionary:=nil;
      end;


    procedure TFPCMakeSection.AddLine(const s:string);
      begin
        if FList=nil then
         raise Exception.Create(Format(s_not_list_sec,[Name]));
        FList.Add(s);
      end;


    procedure TFPCMakeSection.AddKey(const k,v:string);
      begin
        if FDictionary=nil then
         raise Exception.Create(Format(s_not_key_value_sec,[Name]));
        { Don't add empty values }
        if v<>'' then
         FDictionary.Add(k,v);
      end;


    function TFPCMakeSection.GetKey(const k:string):string;
      begin
        if FDictionary=nil then
         raise Exception.Create(Format(s_not_key_value_sec,[Name]));
        GetKey:=FDictionary[k];
      end;


    procedure TFPCMakeSection.ParseIni;
      var
        p : TKeyValueItem;
        i,j,len,maxi : integer;
        s,newkey,value : string;
      begin
        { If already processed skip }
        if assigned(FDictionary) then
         exit;
        { Don't process rules section }
        if (Name='prerules') or (Name='rules') then
         exit;
        { Parse the section }
        FDictionary:=TKeyValue.Create;
        { Parse the list }
        maxi:=FList.Count;
        i:=0;
        while (i<maxi) do
         begin
           s:=Trim(FList[i]);
           len:=Length(s);
           { Concat lines ending with \ }
           while s[len]='\' do
            begin
              Delete(s,len,1);
              if i+1<maxi then
               begin
                 inc(i);
                 s:=s+Trim(FList[i]);
                 len:=Length(s);
               end;
            end;
           { Parse key=value line }
           j:=0;
           while (j<len) and (s[j+1] in ['A'..'Z','a'..'z','0'..'9','_']) do
            inc(j);
           NewKey:=Copy(s,1,j);
           While (j<len) and (s[j+1] in [' ',#9]) do
            inc(j);
           inc(j);
           if s[j]<>'=' then
            Raise Exception.Create(Format(s_err_not_key_value,[s]));
           While (j<len) and (s[j+1] in [' ',#9]) do
            inc(j);
           Value:=Copy(s,j+1,len-j);
           p:=TKeyValueItem(FDictionary.Search(NewKey));
           { Concat values if key already exists }
           if assigned(p) then
            AddToken(p.FValue,Value,' ')
           else
            FDictionary.Add(NewKey,Value);
           inc(i);
         end;
        { List is not used anymore }
        FList.Free;
        FList:=nil;
      end;



    procedure TFPCMakeSection.BuildIniDic(p:TDictionaryItem);
      begin
        with TKeyValueItem(p) do
         begin
           FList.Add(Name+'='+Value);
         end;
      end;


    procedure TFPCMakeSection.BuildIni;
      begin
        if assigned(FList) then
         exit;
        FList:=TStringList.Create;
        FDictionary.Foreach(@BuildIniDic);
        FDictionary.Free;
        FDictionary:=nil;
      end;


    procedure TFPCMakeSection.BuildMakefileDic(p:TDictionaryItem);
      begin
        FList.Add(Uppercase(Name+'_'+TKeyValueItem(p).Name)+'='+TKeyValueItem(p).Value);
      end;


    procedure TFPCMakeSection.BuildMakefile;
      begin
        if assigned(FList) then
         exit;
        FList:=TStringList.Create;
        FDictionary.Foreach(@BuildMakefileDic);
        FDictionary.Free;
        FDictionary:=nil;
      end;


{****************************************************************************
                                   TFPCMake
****************************************************************************}

    constructor TFPCMake.Create(const AFileName:string);
      begin
        FFileName:=AFileName;
        FStream:=nil;
        Init;
      end;


    constructor TFPCMake.CreateFromStream(s:TStream;const AFileName:string);
      begin
        FFileName:=AFileName;
        FStream:=s;
        Init;
      end;


    procedure TFPCMake.Init;
      var
        t : tos;
        c : tcpu;
      begin
        FSections:=TDictionary.Create;
        for c:=low(tcpu) to high(tcpu) do
         for t:=low(tos) to high(tos) do
          FRequireList[c,t]:=TStringList.Create;
        FVariables:=TKeyValue.Create;
        FCommentChars:=[';','#'];
        FEmptyLines:=false;
        FIsPackage:=false;
        FPackageName:='';
        FPackageVersion:='';
        FPackageSec:=nil;
        FExportSec:=nil;
        FillChar(FIncludeTargets,sizeof(FIncludeTargets),true);
        VerboseIdent:='';
        FUsesLCL:=false;
      end;


    destructor TFPCMake.Destroy;
      var
        t : tos;
        c : tcpu;
      begin
        FSections.Free;
        for c:=low(tcpu) to high(tcpu) do
         for t:=low(tos) to high(tos) do
          FRequireList[c,t].Free;
        FVariables.Free;
      end;


    procedure TFPCMake.LoadSections;
      var
        SLInput : TStringList;
        i,j,n : integer;
        s,
        SecName : string;
        CurrSec : TFPCMakeSection;
      begin
        try
          CurrSec:=nil;
          SLInput:=TStringList.Create;
          if assigned(FStream) then
           SLInput.LoadFromStream(FStream)
          else
           SLInput.LoadFromFile(FFileName);
          { Load Input into sections list }
          n:=SLInput.Count;
          i:=0;
          while (i<n) do
           begin
             s:=Trim(SLInput[i]);
             if (EmptyLines and (s='')) or
                ((s<>'') and not(s[1] in FCommentChars)) then
              begin
                { section start? }
                if (s<>'') and (s[1]='[') then
                 begin
                   j:=pos(']',s);
                   if j=0 then
                    raise Exception.Create(Format(s_err_section_start,[FFileName,i+1]));
                   SecName:=Copy(s,2,j-2);
                   CurrSec:=TFPCMakeSection(FSections[SecName]);
                   if CurrSec=nil then
                    CurrSec:=TFPCMakeSection(FSections.Insert(TFPCMakeSection.Create(SecName)));
                 end
                else
                 begin
                   if CurrSec=nil then
                    raise Exception.Create(Format(s_err_no_section,[FFileName,i+1]));
                   { Insert string without spaces stripped }
                   CurrSec.AddLine(SLInput[i]);
                 end;
              end;
             inc(i);
           end;
        finally
          SLInput.Free;
        end;
      end;


    function TFPCMake.CopySection(Sec:TFPCMakeSection;Secname:string):TFPCMakeSection;
      begin
        Result:=TFPCMakeSection(FSections[SecName]);
        if Sec=Nil then
         exit;
        { Clear old section or if not existing create new }
        if assigned(Result) then
         Result.Clear
        else
         Result:=TFPCMakeSection(FSections.Insert(TFPCMakeSection.Create(SecName)));
        Sec.BuildIni;
        Result.List.AddStrings(Sec.List);
        Result.ParseIni;
        Sec.ParseIni;
      end;


    procedure TFPCMake.LoadMakefileFPC;
{$ifdef SupportLCL}
      var
        s : string;
{$endif SupportLCL}
      begin
        LoadSections;
        { Parse all sections }
        FSections.Foreach(@ParseSec);
        { Load package section }
        LoadPackageSection;
        { Add some default variables like FPCDIR, UNITSDIR }
        AddFPCDefaultVariables;
        { Load LCL code ? }
{$ifdef SupportLCL}
        s:=GetVariable('require_packages',true);
        if (pos('lcl',s)>0) or (PackageName='lcl') then
         begin
           FUsesLCL:=true;
           AddLCLDefaultVariables;
         end;
{$endif SupportLCL}
        { Show globals }
        Verbose(FPCMakeDebug,s_globals);
        Variables.Foreach(@PrintDic);
        { Load required packages }
        LoadRequireSection;
      end;


    procedure TFPCMake.Verbose(lvl:TFPCMakeVerbose;const s:string);
      begin
        writeln(VerboseIdent,s);
      end;


    procedure TFPCMake.SetTargets(const s:string);
      var
        hslst : string;
        hs : string;
        hcpu,htarget : string;
        t  : TOs;
        c  : TCpu;
        i  : integer;
        targetset : boolean;
      begin
        FillChar(FIncludeTargets,sizeof(FIncludeTargets),0);
        targetset:=false;
        hslst:=s;
        repeat
          hs:=LowerCase(GetToken(hslst,','));
          if hs='' then
           break;
          { target 'all' includes all targets }
          if hs='all' then
           begin
             for c:=low(TCpu) to high(TCpu) do
              for t:=low(TOs) to high(TOs) do
               if OSCpuPossible[t,c] then
                FIncludeTargets[c,t]:=true;
             targetset:=true;
             break;
           end;
          { full cpu-target specified? }
          i:=pos('-',hs);
          if i>0 then
            begin
              hcpu:=copy(hs,1,i-1);
              htarget:=copy(hs,i+1,length(hs)-i);
              for c:=low(TCpu) to high(TCpu) do
                begin
                  if hcpu=CpuStr[c] then
                    begin
                      for t:=low(TOs) to high(TOs) do
                        begin
                          if htarget=OSStr[t] then
                            begin
                              if OSCpuPossible[t,c] then
                                begin
                                  FIncludeTargets[c,t]:=true;
                                  targetset:=true;
                                end;
                              break;
                            end;
                        end;
                      break;
                    end;
                end;
            end
          else
            begin
              for c:=low(TCpu) to high(TCpu) do
                begin
                  for t:=low(TOS) to high(TOS) do
                    begin
                      if hs=OSStr[t] then
                        begin
                          if OSCpuPossible[t,c] then
                            begin
                              FIncludeTargets[c,t]:=true;
                              targetset:=true;
                            end;
                          break;
                        end;
                    end;
                end;
            end;
        until false;
        if not targetset then
          raise Exception.Create(s_no_targets_set)
        else
         begin
           hs:='';
           for c:=low(TCpu) to high(TCpu) do
            for t:=low(TOs) to high(TOs) do
             if FIncludeTargets[c,t] then
              AddToken(hs,CpuStr[c]+'-'+OSStr[t],' ');
           Verbose(FPCMakeDebug,Format(s_targets_info,[hs]));
         end;
      end;


    procedure TFPCMake.LoadPackageSection;
      var
        s : string;
      begin
        { Get package info from package section }
        FPackageSec:=TFPCMakeSection(FSections['package']);
        if FPackageSec=nil then
         exit;
        { Parse the section to key=value pairs }
        FPackageSec.ParseIni;
        { Are we a subpart of a package, then load that package }
        s:=FPackageSec['main'];
        if s<>'' then
         begin
           SetVariable('package_name',s,false);
           FPackageName:=s;
         end
        else
         begin
           { mandatory name }
           FPackageName:=FPackageSec['name'];
           if FPackageName='' then
            Raise Exception.Create(s_no_package_name);
           { mandatory version }
           FPackageVersion:=FPackageSec['version'];
           if FPackageVersion='' then
            Raise Exception.Create(s_no_package_version);
           FIsPackage:=true;
           { Set the ExportSec }
           FExportSec:=TFPCMakeSection(FSections[Lowercase(FPackageName)]);
         end;
      end;


    procedure TFPCMake.CreateExportSection;
      var
        t : TOS;
        c : TCpu;
      begin
        { Don't create a section twice }
        if FExportSec<>nil then
         exit;
        { Look if we've already an own section, else create a new
          key-value section }
        FExportSec:=TFPCMakeSection(FSections[LowerCase(FPackageName)]);
        if FExportSec=nil then
         FExportSec:=TFPCMakeSection(FSections.Insert(TFPCMakeSection.CreateKeyValue(LowerCase(FPackageName))));
        { Add default the values to the export section }
        FExportSec.AddKey('name',FPackageName);
        FExportSec.AddKey('version',FPackageVersion);
        { Add required packages }
        for c:=low(TCpu) to high(TCpu) do
         for t:=low(TOS) to high(TOS) do
          FExportSec.AddKey('require'+CpuSuffix[c]+OSSuffix[t],FPackageSec['require'+CpuSuffix[c]+OSSuffix[t]]);
        { Unit dir }
        {FExportSec.AddKey('unitdir','$(UNITSDIR)/'+Lowercase(PackageName));}
      end;


    procedure TFPCMake.LoadRequiredPackage(c:TCpu;t:TOS;const ReqName,ReqVersion:string);

        function TryFile(const fn:string):boolean;
        var
          ReqFPCMake : TFPCMake;
        begin
          TryFile:=false;
          if FileExists(fn) then
           begin
             VerboseIdent:=VerboseIdent+'  ';
             Verbose(FPCMakeDebug,'Package '+ReqName+': '+fn);

             ReqFPCMake:=TFPCMake.Create(fn);
             ReqFPCMake.LoadSections;
             ReqFPCMake.LoadPackageSection;
             { Check package name and version }
             if LowerCase(ReqFPCMake.PackageName)<>ExtractFileName(ReqName) then
              raise Exception.Create(Format(s_wrong_package_name,[ReqName,LowerCase(ReqFPCMake.PackageName)]));
             if (ReqVersion<>'') and (ReqFPCMake.PackageVersion<ReqVersion) then
              raise Exception.Create(Format(s_wrong_package_version,[ReqVersion,ReqFPCMake.PackageVersion]));
             { First load the requirements of this package }
             LoadRequires(c,t,ReqFPCMake);
             { Get a copy of the package section }
             CopySection(ReqFPCMake.PackageSec,ReqName+'_package');
             { Get a copy of the export section }
             CopySection(ReqFPCMake.ExportSec,ReqName);
             { Get a copy of the require section }
             CopySection(TFPCMakeSection(ReqFPCMake['require']),ReqName+'_require');
             { Free }
             ReqFPCMake.Free;
             Delete(VerboseIdent,1,2);
             TryFile:=true;
           end;
        end;

      var
        s : string;
      begin
        { Force the current target }
        SetVariable('OSTARGET',OSStr[t],false);
        SetVariable('CPUTARGET',CpuStr[c],false);
{$ifdef LIMIT83}
        SetVariable('FULLTARGET',OSStr[t],false);
{$else}
        SetVariable('FULLTARGET',CpuStr[c]+'-'+OSStr[t],false);
{$endif}
        { Check for Makefile.fpc }
        s:=SubstVariables('$(addsuffix /'+ReqName+'/Makefile.fpc,$(FPCDIR)) $(addsuffix /'+ReqName+'/Makefile.fpc,$(PACKAGESDIR)) $(addsuffix /'+ReqName+'/Makefile.fpc,$(REQUIRE_PACKAGESDIR))');
        Verbose(FPCMakeDebug,'Package "'+ReqName+'": Looking for Makefile.fpc: "'+s+'"');
        s:=SubstVariables('$(firstword $(wildcard '+s+'))');
        if TryFile(s) then
         exit;
        { Check for Package.fpc }
        s:=SubstVariables('$(addsuffix /'+ReqName+'/Package.fpc,$(FPCDIR)) $(addsuffix /'+ReqName+'/Package.fpc,$(UNITSDIR)) $(addsuffix /'+ReqName+'/Package.fpc,$(REQUIRE_UNITSDIR))');
        Verbose(FPCMakeDebug,'Package "'+ReqName+'": Looking for Package.fpc: "'+s+'"');
        s:=SubstVariables('$(firstword $(wildcard '+s+'))');
        if TryFile(s) then
         exit;
        Raise Exception.Create(Format(s_package_not_found,[OSStr[t],Reqname]));
      end;


    procedure TFPCMake.LoadRequiredDir(c:TCpu;t:TOS;const MainPack,currdir,subdir:string);
        var
          ReqFPCMake : TFPCMake;
          s : string;
        begin
          VerboseIdent:=VerboseIdent+'  ';
          s:=currdir+subdir;
          Verbose(FPCMakeDebug,'Subdir: '+s+'/Makefile.fpc');
          if not FileExists(s+'/Makefile.fpc') then
           begin
             { give better error what is wrong }
             if not PathExists(s) then
              Raise Exception.Create(Format(s_directory_not_found,[s]))
             else
              Raise Exception.Create(Format(s_makefilefpc_not_found,[s]));
           end;
          { Process Makefile.fpc }
          ReqFPCMake:=TFPCMake.Create(currdir+subdir+'/Makefile.fpc');
          ReqFPCMake.LoadSections;
          ReqFPCMake.LoadPackageSection;
          { Are we a subpackage? }
          if (ReqFPCMake.GetVariable('package_name',false)<>MainPack) then
           begin
             ReqFPCMake.Free;
             Delete(VerboseIdent,1,2);
             exit;
           end;
          { Load the requirements of this package }
          LoadRequires(c,t,ReqFPCMake);
          { Add the current requirements to our parents requirements }
          s:=ReqFPCMake.GetTargetVariable(c,t,'require_packages',true);
          SetVariable('require_packages'+OSSuffix[t]+cpusuffix[c],s,true);
          if ReqFPCMake.GetVariable('require_libc',false)<>'' then
           SetVariable('require_libc','y',false);
          { Free }
          ReqFPCMake.Free;
          Delete(VerboseIdent,1,2);
        end;


    procedure TFPCMake.LoadRequires(c:TCpu;t:TOS;FromFPCMake:TFPCMake);
      var
        s,
        ReqDir,
        ReqName,
        ReqVersion : string;
        i,j : integer;
      begin
        { packages }
        s:=FromFPCMake.GetTargetVariable(c,t,'require_packages',true);
        Verbose(FPCMakeDebug,'Required packages for '+OSStr[t]+'-'+CpuStr[c]+': '+s);
        repeat
          reqname:=GetToken(s,' ');
          if reqname='' then
           break;
          i:=Pos('(',ReqName);
          if i>0 then
           begin
             j:=Pos(')',ReqName);
             if (i=1) or (j=0) then
              Raise Exception.Create(Format(s_err_require_format,[ReqName]));
             ReqVersion:=Copy(ReqName,i+1,j-i-1);
             ReqName:=Copy(ReqName,1,i-1);
           end
         else
           ReqVersion:='';
          { We only use lowercase names }
          ReqName:=Lowercase(ReqName);
          { Already loaded ? }
         if (RequireList[c,t].IndexOf(ReqName)=-1) then
           begin
             LoadRequiredPackage(c,t,ReqName,ReqVersion);
             RequireList[c,t].Add(ReqName);
           end;
        until false;
        { sub dirs }
        s:=FromFPCMake.GetTargetVariable(c,t,'target_dirs',true);
        Verbose(FPCMakeDebug,'Required dirs for '+CpuStr[c]+'-'+OSStr[t]+': '+s);
        repeat
          reqdir:=GetToken(s,' ');
          if reqdir='' then
           break;
          LoadRequiredDir(c,t,FromFPCMake.FPackageName,ExtractFilePath(FromFPCMake.FFileName),ReqDir)
        until false;
      end;


    procedure TFPCMake.LoadRequireSection;
      var
        s : string;
        t : tos;
        c : tcpu;
      begin
        { Check FPCMake version }
        s:=GetVariable('require_fpcmake',false);
        if (s>version) then
         raise Exception.Create(Format(s_fpcmake_version_required,[s]));
        { Maybe add an implicit rtl dependency if there is something
          to compile }
        s:=GetVariable('require_packages',false);
        if (GetVariable('require_nortl',false)='') and
           (HasTargetVariable('target_programs') or
            HasTargetVariable('target_units') or
            HasTargetVariable('target_examples')) and
           (Pos('rtl(',s)=0) and (getvariable('package_name',false)<>'rtl') then
         begin
           s:='rtl '+s;
           SetVariable('require_packages',s,false);
         end;
        { Load recursively all required packages starting with this Makefile.fpc }
        for c:=low(TCpu) to high(TCpu) do
          for t:=low(Tos) to high(Tos) do
            if FIncludeTargets[c,t] then
              LoadRequires(c,t,self);
      end;


    function TFPCMake.GetTargetRequires(c:TCpu;t:Tos):TStringList;
      var
        ReqSec  : TFPCMakeSection;
        ReqList : TStringList;

        procedure AddReqSec(c:TCpu;t:Tos;Sec:TFPCMakeSection);
        var
          s,
          ReqName : string;
          RSec : TFPCMakeSection;
          i : integer;
        begin
          s:=Sec['packages']+' '+
             Sec['packages'+CpuSuffix[c]]+' '+
             Sec['packages'+OSSuffix[t]]+' '+
             Sec['packages'+OSSuffix[t]+CpuSuffix[c]];
          repeat
            ReqName:=GetToken(s,' ');
            if ReqName='' then
             break;
            i:=Pos('(',ReqName);
            if i>0 then
             ReqName:=Copy(ReqName,1,i-1);
            { We only use lowercase names }
            ReqName:=Lowercase(ReqName);
            { Already loaded ? }
            if (ReqList.IndexOf(ReqName)=-1) then
             begin
               RSec:=TFPCMakeSection(FSections[ReqName+'_require']);
               if assigned(RSec) then
                AddReqSec(c,t,RSec);
               ReqList.Add(ReqName);
             end;
          until false;
        end;

      begin
        ReqList:=TStringList.Create;
        ReqSec:=TFPCMakeSection(FSections['require']);
        { Building fpmake itself always requires the rtl }
        if HasTargetVariable('target_fpmake') then
         ReqList.Add('rtl');
        if assigned(ReqSec) then
         AddReqSec(c,t,ReqSec);
        GetTargetRequires:=ReqList;
      end;


    function TFPCMake.CheckLibcRequire:boolean;
      var
        i : integer;
        RSec : TFPCMakeSection;
        t : tos;
        c : tcpu;
      begin
        Result:=false;
        if GetVariable('require_libc',false)<>'' then
         begin
           Result:=true;
           exit;
         end;
        for c:=low(tcpu) to high(tcpu) do
          for t:=low(tos) to high(tos) do
            if FIncludeTargets[c,t] then
              begin
                for i:=0 to RequireList[c,t].Count-1 do
                 begin
                   RSec:=TFPCMakeSection(FSections[RequireList[c,t][i]+'_require']);
                   if assigned(RSec) then
                    begin
                      if RSec['libc']<>'' then
                       begin
                         Result:=true;
                         exit;
                       end;
                    end;
                 end;
              end;
      end;


    procedure TFPCMake.AddFPCDefaultVariables;
      var
        cpu : TCpu;
        hs,s : string;
      begin
        { Already set FPCDIR }
        hs:='';
        s:=GetVariable('FPCDIR',false);
        if s<>'' then
         hs:=SubstVariables('$(wildcard $(addprefix '+s+'/,rtl units))');
        { Load from environment }
        if hs='' then
         begin
           s:=GetEnv('FPCDIR');
           if s<>'' then
            hs:=SubstVariables('$(wildcard $(addprefix '+s+'/,rtl units))');
         end;
        { default_fpcdir }
        if hs='' then
         begin
           s:=GetVariable('default_fpcdir',true);
           { add the current subdir to relative paths }
           if s<>'' then
            begin
{$ifdef UNIX}
              if (s[1]<>'/') then
{$else}
              if ((length(s)>2) and (s[2]<>':')) or (length(s)<=2) then
{$endif}
               s:=ExtractFilePath(FFileName)+s;
              hs:=SubstVariables('$(wildcard $(addprefix '+s+'/,rtl units))');
            end
         end;
        { OS defaults }
        if hs='' then
         begin
{$ifdef UNIX}
{$ifndef NO_UNIX_UNIT}
           cpu := low(TCpu);
           while(cpuStr[cpu] <> {$I %FPCTARGETCPU%}) do begin
             Inc(cpu);
             if cpu > high(TCpu) then
               raise Exception.Create('Internal error');
           end;
           if FileExists('/usr/local/bin/ppc' + ppcSuffix[cpu]) then
            begin
              s:=ExtractFilePath({$ifdef ver1_0}ReadLink{$else}fpReadlink{$endif}('/usr/local/bin/ppc' + ppcSuffix[cpu]));
              if s<>'' then
               begin
                 if s[length(s)]='/' then
                  delete(s,length(s),1);
                 hs:=SubstVariables('$(wildcard $(addprefix '+s+'/,rtl units))');
               end;
            end;
           if hs='' then
            begin
              if FileExists('/usr/bin/ppc' + ppcSuffix[cpu]) then
               begin
                 s:=ExtractFilePath({$ifdef ver1_0}ReadLink{$else}fpReadLink{$endif}('/usr/bin/ppc' + ppcSuffix[cpu]));
                 if s<>'' then
                  begin
                    if s[length(s)]='/' then
                     delete(s,length(s),1);
                    hs:=SubstVariables('$(wildcard $(addprefix '+s+'/,rtl units))');
                  end;
               end;
            end;
{$endif}
{$else UNIX}
           hs:=ExtractFilePath(FSearch('ppc386.exe',getenv('PATH')));
           if hs<>'' then
            begin
              s:=hs+'/..';
              hs:=SubstVariables('$(wildcard $(addprefix '+s+'/,rtl units))');
              if hs='' then
               begin
                 s:=s+'/..';
                 hs:=SubstVariables('$(wildcard $(addprefix '+s+'/,rtl units))');
               end;
            end;
           if hs='' then
            s:='c:/pp';
{$endif UNIX}
         end;
        SetVariable('FPCDIR',s,false);
        { PACKAGESDIR }
        if GetVariable('PACKAGESDIR',false)='' then
         SetVariable('PACKAGESDIR','$(FPCDIR) $(FPCDIR)/packages',false);
        { UNITSDIR }
        if GetVariable('UNITSDIR',false)='' then
         SetVariable('UNITSDIR','$(FPCDIR)/units/$(FULLTARGET)',false);
        { BASEDIR }
        SetVariable('BASEDIR',GetCurrentDir,false);
      end;


    procedure TFPCMake.AddLCLDefaultVariables;
      var
        hs,s : string;
      begin
        { Already set LCLDIR }
        hs:=SubstVariables('$(wildcard $(LCLDIR)/units)');
        { Load from environment }
        if hs='' then
         begin
           SetVariable('LCLDIR',GetEnv('LCLDIR'),false);
           hs:=SubstVariables('$(wildcard $(LCLDIR)/units)');
         end;
        { default_lcldir }
        if hs='' then
         begin
           s:=GetVariable('default_lcldir',true);
           { add the current subdir to relative paths }
           if s<>'' then
            begin
{$ifdef UNIX}
              if (s[1]<>'/') then
{$else}
              if ((length(s)>2) and (s[2]<>':')) or (length(s)<=2) then
{$endif}
               s:=ExtractFilePath(FFileName)+s;
              SetVariable('LCLDIR',s,false);
              hs:=SubstVariables('$(wildcard $(LCLDIR)/units)');
            end
         end;
        { OS defaults }
        if hs='' then
         begin
           hs:=SubstVariables('$(subst /units,,$(firstword $(wildcard $(addsuffix /units,$(BASEDIR)/lcl $(BASEDIR)))))');
           if hs<>'' then
            SetVariable('LCLDIR',hs,false);
         end;
{$ifdef UNIX}
        if hs='' then
         begin
           hs:=SubstVariables('$(subst /units,,$(firstword $(wildcard $(addsuffix /lib/lazarus/units,/usr/local /usr))))');
           if hs<>'' then
            SetVariable('LCLDIR',hs,false);
         end;
{$endif UNIX}
        { Add components to PACKAGESDIR }
        SetVariable('PACKAGESDIR',SubstVariables('$(wildcard $(LCLDIR)/.. $(LCLDIR)/../components $(LCLDIR)/components)'),true);
      end;


    function TFPCMake.SubstVariables(const s:string):string;

      function Expect(var s:string;c:char):boolean;
      begin
        if (s<>'') and (s[1]=c) then
         begin
           Delete(s,1,1);
           Result:=true;
         end
        else
         begin
           Verbose(FPCMakeError,'Error "'+c+'" expected');
           Result:=false;
         end;
      end;

      function GetVar(var s:string;untilc:char):string;
      var
        i,j,k : integer;
        first : boolean;
        func,
        tok,s1,s2,s3 : string;
        Sec   : TFPCMakeSection;
      begin
        Result:='';
        repeat
          j:=Pos(untilc,s);
          if j=0 then
           j:=Length(s)+1;
          i:=Pos('$(',s);
          if (j<i) or (i=0) then
           break;
          Result:=Result+Copy(s,1,i-1);
          Delete(s,1,i+1);
          { Maybe Function ? }
          j:=Pos(')',s);
          if j=0 then
           j:=Length(s)+1;
          i:=Pos(' ',s);
          if i=0 then
           i:=Length(s)+1;
          if i<j then
           begin
             { It's a function }
             Func:=Copy(s,1,i-1);
//writeln('func: ',func);
             { $(wildcard <list>) }
             if Func='wildcard' then
              begin
                Delete(s,1,9);
                s1:=GetVar(s,')');
                Expect(s,')');
                first:=true;
                repeat
                  tok:=GetToken(s1,' ');
                  if tok='' then
                   break;
                  if PathOrFileExists(tok) then
                   begin
                     if not first then
                      Result:=Result+' '
                     else
                      first:=false;
                     Result:=Result+tok;
                   end;
                until false;
              end
             { $(addprefix <suffix>,<list>) }
             else if Func='addprefix' then
              begin
                Delete(s,1,10);
                s1:=GetVar(s,',');
                if Expect(s,',') then
                 begin
                   s2:=GetVar(s,')');
                   Expect(s,')');
                 end;
                first:=true;
                repeat
                  tok:=GetToken(s2,' ');
                  if tok='' then
                   break;
                  if not first then
                   Result:=Result+' '
                  else
                   first:=false;
                  Result:=Result+s1+tok;
                until false;
              end
             { $(addsuffix <suffix>,<list>) }
             else if Func='addsuffix' then
              begin
                Delete(s,1,10);
                s1:=GetVar(s,',');
                if Expect(s,',') then
                 begin
                   s2:=GetVar(s,')');
                   Expect(s,')');
                 end;
                first:=true;
                repeat
                  tok:=GetToken(s2,' ');
                  if tok='' then
                   break;
                  if not first then
                   Result:=Result+' '
                  else
                   first:=false;
                  Result:=Result+tok+s1;
                until false;
              end
             { $(firstword <list>) }
             else if Func='firstword' then
              begin
                Delete(s,1,10);
                s1:=GetVar(s,')');
                Expect(s,')');
                Result:=GetToken(s1,' ');
              end
             { $(subst <oldpat>,<newpat>,<string>) }
             else if Func='subst' then
              begin
                Delete(s,1,6);
                s1:=GetVar(s,',');
                if Expect(s,',') then
                 begin
                   s2:=GetVar(s,',');
                   if Expect(s,',') then
                    begin
                      s3:=GetVar(s,')');
                      Expect(s,')');
                    end;
                 end;
                Result:=StringReplace(s3,s1,s2,[rfReplaceAll]);
              end;
           end
          else
           begin
             s2:=Copy(s,1,j-1);
             Delete(s,1,j);
             k:=pos('_',s2);
             if k>0 then
              begin
                s3:=LowerCase(Copy(s2,k+1,Length(s2)-k));
                s2:=LowerCase(Copy(s2,1,k-1));
                Sec:=TFPCMakeSection(Section[s2]);
                if assigned(Sec) then
                 s2:=Sec[s3]
                else
                 s2:='';
              end
             else
              s2:=Variables[s2];
             Insert(s2,s,1);
           end;
        until false;
        Result:=Result+Copy(s,1,j-1);
        Delete(s,1,j-1);
      end;

      var
        s1 : string;
      begin
//writeln('S: ',s);
        s1:=s;
        Result:=GetVar(s1,#0);
//writeln('R: ',result);
      end;


    function TFPCMake.GetVariable(const inivar:string;dosubst:boolean):string;
      var
        Sec : TFPCMakeSection;
        Dic : TKeyValue;
        i   : integer;
      begin
        Result:='';
        i:=Pos('_',inivar);
        if i<>0 then
         begin
           Sec:=TFPCMakeSection(FSections[Copy(Inivar,1,i-1)]);
           if assigned(Sec) then
            begin
              if not assigned(Sec.Dictionary) then
               Sec.ParseIni;
              Dic:=TKeyValue(Sec.Dictionary);
              Result:=Dic[Copy(IniVar,i+1,Length(IniVar)-i)];
            end
           else
            exit;
         end
        else
         Result:=Variables[IniVar];
        { Substition asked ? }
        if dosubst then
         Result:=SubstVariables(Result);
      end;


    function TFPCMake.GetTargetVariable(c:TCPU;t:TOS;const inivar:string;dosubst:boolean):string;
      begin
        result:=Trim(GetVariable(inivar,dosubst)+' '+
                     GetVariable(inivar+cpusuffix[c],dosubst)+' '+
                     GetVariable(inivar+OSSuffix[t],dosubst)+' '+
                     GetVariable(inivar+CpuSuffix[c]+OSSuffix[t],dosubst)+' '+
                     GetVariable(inivar+OSSuffix[t]+cpusuffix[c],dosubst));
      end;


    function TFPCMake.HasVariable(const inivar:string):boolean;
      begin
        Result:=(GetVariable(IniVar,false)<>'');
      end;


    function TFPCMake.HasTargetVariable(const inivar:string):boolean;
      var
        c:TCPU;
        t:TOS;
      begin
        result:=false;
        for c:=low(tcpu) to high(tcpu) do
          for t:=low(tos) to high(tos) do
           if FIncludeTargets[c,t] then
             begin
               if (GetVariable(inivar,false)<>'') or
                  (GetVariable(inivar+cpusuffix[c],false)<>'') or
                  (GetVariable(inivar+OSSuffix[t],false)<>'') or
                  (GetVariable(inivar+CpuSuffix[c]+OSSuffix[t],false)<>'') or
                  (GetVariable(inivar+OSSuffix[t]+cpusuffix[c],false)<>'') then
                 begin
                   result:=true;
                   exit;
                 end;
              end;
      end;


    function TFPCMake.SetVariable(const inivar,value:string;add:boolean):string;
      var
        Sec : TFPCMakeSection;
        P   : TKeyValueItem;
        i   : integer;
        tempval,key : string;
      begin
        Result:='';
        i:=Pos('_',inivar);
        if i<>0 then
         begin
           Sec:=TFPCMakeSection(FSections[Copy(Inivar,1,i-1)]);
           if Sec=nil then
            Sec:=TFPCMakeSection(FSections.Insert(TFPCMakeSection.CreateKeyValue(Copy(Inivar,1,i-1))));
           key:=Copy(IniVar,i+1,Length(IniVar)-i);
           p:=TKeyValueItem(Sec.Dictionary.Search(Key));
           if assigned(p) then
            begin
              if Add then
               AddToken(p.FValue,Value,' ')
              else
               p.Value:=Value;
            end
           else
            TKeyValue(Sec.Dictionary).Add(key,value);
         end
        else
         begin
           if Add then
            begin
              tempval:=Variables[IniVar];
              AddToken(tempval,Value,' ');
              Variables[IniVar]:=tempval;
            end
           else
            Variables[IniVar]:=Value;
         end;
      end;


    procedure TFPCMake.ParseSec(p:TDictionaryItem);
      begin
        TFPCMakeSection(p).ParseIni;
      end;


    procedure TFPCMake.PrintSec(p:TDictionaryItem);
      var
        i : integer;
      begin
        with TFPCMakeSection(p) do
         begin
           Verbose(FPCMakeDebug,'['+Name+']');
           if assigned(FList) then
            begin
              Verbose(FPCMakeDebug,'  List:');
              for i:=0 to FList.Count-1 do
               Verbose(FPCMakeDebug,'   "'+FList[i]+'"');
              if assigned(FDictionary) then
               Verbose(FPCMakeDebug,'');
            end;
           if assigned(FDictionary) then
            begin
              Verbose(FPCMakeDebug,'  Dictionary:');
              FDictionary.Foreach(@PrintDic);
            end;
         end;
      end;


    procedure TFPCMake.PrintDic(p:TDictionaryItem);
      begin
        with TKeyValueItem(p) do
         begin
           Verbose(FPCMakeDebug,'   '+name+' = "'+value+'"');
         end;
      end;


    procedure TFPCMake.Print;
      begin
        { global variables }
        Verbose(FPCMakeDebug,'[global variables]');
        Verbose(FPCMakeDebug,'  Dictionary:');
        Variables.Foreach(@PrintDic);
        { sections }
        FSections.Foreach(@PrintSec);
      end;


    function TFPCMake.GetSec(const AName:string):TDictionaryItem;
      begin
        GetSec:=FSections.Search(AName);
      end;

end.
