{
    Copyright (c) 1998-2002 by Florian Klaempfl and Peter Vreman

    This module provides some basic file/dir handling utils and classes

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
unit cfileutl;

{$i fpcdefs.inc}

{$define usedircache}

interface

    uses
{$ifdef hasunix}
      Baseunix,unix,
{$endif hasunix}
{$ifdef win32}
      Windows,
{$endif win32}
{$if defined(go32v2) or defined(watcom)}
      Dos,
{$endif}
{$ifdef macos}
      macutils,
{$endif macos}
{$IFNDEF USE_FAKE_SYSUTILS}
      SysUtils,
{$ELSE}
      fksysutl,
{$ENDIF}
      GlobType,
      CUtils,CClasses,
      Systems;

    type
      TCachedDirectory = class(TFPHashObject)
      private
        FDirectoryEntries : TFPHashList;
        FCached : Boolean;
        procedure FreeDirectoryEntries;
        function GetItemAttr(const AName: TCmdStr): longint;
        function TryUseCache: boolean;
        procedure ForceUseCache;
        procedure Reload;
      public
        constructor Create(AList:TFPHashObjectList;const AName:TCmdStr);
        destructor  destroy;override;
        function FileExists(const AName:TCmdStr):boolean;
        function FileExistsCaseAware(const path, fn: TCmdStr; out FoundName: TCmdStr):boolean;
        function DirectoryExists(const AName:TCmdStr):boolean;
        property DirectoryEntries:TFPHashList read FDirectoryEntries;
      end;

      TCachedSearchRec = record
        Name       : TCmdStr;
        Attr       : byte;
        Pattern    : TCmdStr;
        CachedDir  : TCachedDirectory;
        EntryIndex : longint;
      end;

      PCachedDirectoryEntry =  ^TCachedDirectoryEntry;
      TCachedDirectoryEntry = record
        RealName: TCmdStr;
        Attr    : longint;
      end;

      TDirectoryCache = class
      private
        FDirectories : TFPHashObjectList;
        function GetDirectory(const ADir:TCmdStr):TCachedDirectory;
      public
        constructor Create;
        destructor  destroy;override;
        function FileExists(const AName:TCmdStr):boolean;
        function FileExistsCaseAware(const path, fn: TCmdStr; out FoundName: TCmdStr):boolean;
        function DirectoryExists(const AName:TCmdStr):boolean;
        function FindFirst(const APattern:TCmdStr;var Res:TCachedSearchRec):boolean;
        function FindNext(var Res:TCachedSearchRec):boolean;
        function FindClose(var Res:TCachedSearchRec):boolean;
      end;

      TSearchPathList = class(TCmdStrList)
        procedure AddPath(s:TCmdStr;addfirst:boolean);overload;
        procedure AddLibraryPath(const sysroot: TCmdStr; s:TCmdStr;addfirst:boolean);overload;
        procedure AddList(list:TSearchPathList;addfirst:boolean);
        function  FindFile(const f : TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
      end;

    function  bstoslash(const s : TCmdStr) : TCmdStr;
    {Gives the absolute path to the current directory}
    function  GetCurrentDir:TCmdStr;
    {Gives the relative path to the current directory,
     with a trailing dir separator. E. g. on unix ./ }
    function CurDirRelPath(systeminfo: tsysteminfo): TCmdStr;
    function  path_absolute(const s : TCmdStr) : boolean;
    Function  PathExists (const F : TCmdStr;allowcache:boolean) : Boolean;
    Function  FileExists (const F : TCmdStr;allowcache:boolean) : Boolean;
    function  FileExistsNonCase(const path,fn:TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
    Function  RemoveDir(d:TCmdStr):boolean;
    Function  FixPath(const s:TCmdStr;allowdot:boolean):TCmdStr;
    function  FixFileName(const s:TCmdStr):TCmdStr;
    function  TargetFixPath(s:TCmdStr;allowdot:boolean):TCmdStr;
    function  TargetFixFileName(const s:TCmdStr):TCmdStr;
    procedure SplitBinCmd(const s:TCmdStr;var bstr: TCmdStr;var cstr:TCmdStr);
    function  FindFile(const f : TCmdStr; const path : TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
{    function  FindFilePchar(const f : TCmdStr;path : pchar;allowcache:boolean;var foundfile:TCmdStr):boolean;}
    function  FindFileInExeLocations(const bin:TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
    function  FindExe(const bin:TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
    function  GetShortName(const n:TCmdStr):TCmdStr;
    function maybequoted(const s:string):string;
    function maybequoted(const s:ansistring):ansistring;
    function maybequoted_for_script(const s:ansistring; quote_script: tscripttype):ansistring;

    procedure InitFileUtils;
    procedure DoneFileUtils;

    function UnixRequoteWithDoubleQuotes(const QuotedStr: TCmdStr): TCmdStr;
    function RequotedExecuteProcess(const Path: AnsiString; const ComLine: AnsiString; Flags: TExecuteFlags = []): Longint;
    function RequotedExecuteProcess(const Path: AnsiString; const ComLine: array of AnsiString; Flags: TExecuteFlags = []): Longint;
    function Shell(const command:ansistring): longint;

  { hide Sysutils.ExecuteProcess in units using this one after SysUtils}
  const
    ExecuteProcess = 'Do not use' deprecated 'Use cfileutil.RequotedExecuteProcess instead, ExecuteProcess cannot deal with single quotes as used by Unix command lines';

{ * Since native Amiga commands can't handle Unix-style relative paths used by the compiler,
    and some GNU tools, Unix2AmigaPath is needed to handle such situations (KB) * }

{$IFDEF HASAMIGA}
{ * PATHCONV is implemented in the Amiga/MorphOS system unit * }
{$NOTE TODO Amiga: implement PathConv() in System unit, which works with AnsiString}
function Unix2AmigaPath(path: ShortString): ShortString; external name 'PATHCONV';
{$ELSE}
function Unix2AmigaPath(path: String): String;
{$ENDIF}

{$if FPC_FULLVERSION < 20701}
type
  TRawByteSearchRec = TSearchRec;
{$endif}


implementation

    uses
      Comphook,
      Globals;

{$undef AllFilesMaskIsInRTL}

{$if (FPC_VERSION > 2)}
  {$define AllFilesMaskIsInRTL}
{$endif FPC_VERSION}

{$if (FPC_VERSION = 2) and (FPC_RELEASE > 2)}
  {$define AllFilesMaskIsInRTL}
{$endif}

{$if (FPC_VERSION = 2) and (FPC_RELEASE = 2) and (FPC_PATCH > 0)}
  {$define AllFilesMaskIsInRTL}
{$endif}

{$ifndef AllFilesMaskIsInRTL}
  {$if defined(go32v2) or defined(watcom)}
  const
    AllFilesMask = '*.*';
  {$else}
  const
    AllFilesMask = '*';
  {$endif not (go32v2 or watcom)}
{$endif not AllFilesMaskIsInRTL}
    var
      DirCache : TDirectoryCache;


{$IFNDEF HASAMIGA}
{ Stub function for Unix2Amiga Path conversion functionality, only available in
  Amiga/MorphOS RTL. I'm open for better solutions. (KB) }
function Unix2AmigaPath(path: String): String;
begin
  Unix2AmigaPath:=path;
end;
{$ENDIF}



{****************************************************************************
                           TCachedDirectory
****************************************************************************}

    constructor TCachedDirectory.create(AList:TFPHashObjectList;const AName:TCmdStr);
      begin
        inherited create(AList,AName);
        FDirectoryEntries:=TFPHashList.Create;
        FCached:=False;
      end;


    destructor TCachedDirectory.destroy;
      begin
        FreeDirectoryEntries;
        FDirectoryEntries.Free;
        inherited destroy;
      end;


    function TCachedDirectory.TryUseCache:boolean;
      begin
        Result:=True;
        if FCached then
          exit;
        if not current_settings.disabledircache then
          ForceUseCache
        else
          Result:=False;
      end;


    procedure TCachedDirectory.ForceUseCache;
      begin
        if not FCached then
          begin
            FCached:=True;
            Reload;
          end;
      end;


    procedure TCachedDirectory.FreeDirectoryEntries;
      var
        i: Integer;
      begin
        if not(tf_files_case_aware in source_info.flags) then
          exit;
        for i := 0 to DirectoryEntries.Count-1 do
          dispose(PCachedDirectoryEntry(DirectoryEntries[i]));
      end;


    function TCachedDirectory.GetItemAttr(const AName: TCmdStr): longint;
      var
        entry: PCachedDirectoryEntry;
      begin
        if not(tf_files_case_sensitive in source_info.flags) then
          if (tf_files_case_aware in source_info.flags) then
            begin
              entry:=PCachedDirectoryEntry(DirectoryEntries.Find(Lower(AName)));
              if assigned(entry) then
                Result:=entry^.Attr
              else
                Result:=0;
            end
          else
            Result:=PtrUInt(DirectoryEntries.Find(Lower(AName)))
        else
          Result:=PtrUInt(DirectoryEntries.Find(AName));
      end;


    procedure TCachedDirectory.Reload;
      var
        dir   : TRawByteSearchRec;
        entry : PCachedDirectoryEntry;
      begin
        FreeDirectoryEntries;
        DirectoryEntries.Clear;
        if findfirst(IncludeTrailingPathDelimiter(Name)+AllFilesMask,faAnyFile or faDirectory,dir) = 0 then
          begin
            repeat
              if ((dir.attr and faDirectory)<>faDirectory) or
                 ((dir.Name<>'.') and
                  (dir.Name<>'..')) then
                begin
                  { Force Archive bit so the attribute always has a value. This is needed
                    to be able to see the difference in the directoryentries lookup if a file
                    exists or not }
                  Dir.Attr:=Dir.Attr or faArchive;
                  if not(tf_files_case_sensitive in source_info.flags) then
                    if (tf_files_case_aware in source_info.flags) then
                      begin
                        new(entry);
                        entry^.RealName:=Dir.Name;
                        entry^.Attr:=Dir.Attr;
                        DirectoryEntries.Add(Lower(Dir.Name),entry)
                      end
                    else
                      DirectoryEntries.Add(Lower(Dir.Name),Pointer(Ptrint(Dir.Attr)))
                  else
                    DirectoryEntries.Add(Dir.Name,Pointer(Ptrint(Dir.Attr)));
                end;
            until findnext(dir) <> 0;
            findclose(dir);
          end;
      end;


    function TCachedDirectory.FileExists(const AName:TCmdStr):boolean;
      var
        Attr : Longint;
      begin
        if not TryUseCache then
          begin
            { prepend directory name again }
            result:=cfileutl.FileExists(Name+AName,false);
            exit;
          end;
        Attr:=GetItemAttr(AName);
        if Attr<>0 then
          Result:=((Attr and faDirectory)=0)
        else
          Result:=false;
      end;


    function TCachedDirectory.FileExistsCaseAware(const path, fn: TCmdStr; out FoundName: TCmdStr):boolean;
      var
        entry : PCachedDirectoryEntry;
      begin
        if (tf_files_case_aware in source_info.flags) then
          begin
            if not TryUseCache then
              begin
                Result:=FileExistsNonCase(path,fn,false,FoundName);
                exit;
              end;
            entry:=PCachedDirectoryEntry(DirectoryEntries.Find(Lower(ExtractFileName(fn))));
            if assigned(entry) and
               (entry^.Attr<>0) and
               ((entry^.Attr and faDirectory) = 0) then
              begin
                 FoundName:=ExtractFilePath(path+fn)+entry^.RealName;
                 Result:=true
              end
            else
              Result:=false;
          end
        else
          { should not be called in this case, use plain FileExists }
          Result:=False;
      end;


    function TCachedDirectory.DirectoryExists(const AName:TCmdStr):boolean;
      var
        Attr : Longint;
      begin
        if not TryUseCache then
          begin
            Result:=PathExists(Name+AName,false);
            exit;
          end;
        Attr:=GetItemAttr(AName);
        if Attr<>0 then
          Result:=((Attr and faDirectory)=faDirectory)
        else
          Result:=false;
      end;


{****************************************************************************
                           TDirectoryCache
****************************************************************************}

    constructor TDirectoryCache.create;
      begin
        inherited create;
        FDirectories:=TFPHashObjectList.Create(true);
      end;


    destructor TDirectoryCache.destroy;
      begin
        FDirectories.Free;
        inherited destroy;
      end;


    function TDirectoryCache.GetDirectory(const ADir:TCmdStr):TCachedDirectory;
      var
        CachedDir : TCachedDirectory;
        DirName   : TCmdStr;
      begin
        if ADir='' then
          DirName:='.'+source_info.DirSep
        else
          DirName:=ADir;
        CachedDir:=TCachedDirectory(FDirectories.Find(DirName));
        if not assigned(CachedDir) then
          CachedDir:=TCachedDirectory.Create(FDirectories,DirName);
        Result:=CachedDir;
      end;


    function TDirectoryCache.FileExists(const AName:TCmdStr):boolean;
      var
        CachedDir : TCachedDirectory;
      begin
        Result:=false;
        CachedDir:=GetDirectory(ExtractFilePath(AName));
        if assigned(CachedDir) then
          Result:=CachedDir.FileExists(ExtractFileName(AName));
      end;


    function TDirectoryCache.FileExistsCaseAware(const path, fn: TCmdStr; out FoundName: TCmdStr):boolean;
      var
        CachedDir : TCachedDirectory;
      begin
        Result:=false;
        CachedDir:=GetDirectory(ExtractFilePath(path+fn));
        if assigned(CachedDir) then
          Result:=CachedDir.FileExistsCaseAware(path,fn,FoundName);
      end;


    function TDirectoryCache.DirectoryExists(const AName:TCmdStr):boolean;
      var
        CachedDir : TCachedDirectory;
      begin
        Result:=false;
        CachedDir:=GetDirectory(ExtractFilePath(AName));
        if assigned(CachedDir) then
          Result:=CachedDir.DirectoryExists(ExtractFileName(AName));
      end;


    function TDirectoryCache.FindFirst(const APattern:TCmdStr;var Res:TCachedSearchRec):boolean;
      begin
        Res.Pattern:=ExtractFileName(APattern);
        Res.CachedDir:=GetDirectory(ExtractFilePath(APattern));
        Res.CachedDir.ForceUseCache;
        Res.EntryIndex:=0;
        if assigned(Res.CachedDir) then
          Result:=FindNext(Res)
        else
          Result:=false;
      end;


    function TDirectoryCache.FindNext(var Res:TCachedSearchRec):boolean;
      var
        entry: PCachedDirectoryEntry;
      begin
        if Res.EntryIndex<Res.CachedDir.DirectoryEntries.Count then
          begin
            if (tf_files_case_aware in source_info.flags) then
              begin
                entry:=Res.CachedDir.DirectoryEntries[Res.EntryIndex];
                Res.Name:=entry^.RealName;
                Res.Attr:=entry^.Attr;
              end
            else
              begin
                Res.Name:=Res.CachedDir.DirectoryEntries.NameOfIndex(Res.EntryIndex);
                Res.Attr:=PtrUInt(Res.CachedDir.DirectoryEntries[Res.EntryIndex]);
              end;
            inc(Res.EntryIndex);
            Result:=true;
          end
        else
          Result:=false;
      end;


    function TDirectoryCache.FindClose(var Res:TCachedSearchRec):boolean;
      begin
        { nothing todo }
        result:=true;
      end;


{****************************************************************************
                                   Utils
****************************************************************************}

    function bstoslash(const s : TCmdStr) : TCmdStr;
    {
      return TCmdStr s with all \ changed into /
    }
      var
         i : longint;
      begin
        Result:='';
        setlength(bstoslash,length(s));
        for i:=1to length(s) do
         if s[i]='\' then
          bstoslash[i]:='/'
         else
          bstoslash[i]:=s[i];
      end;


   {Gives the absolute path to the current directory}
     var
       CachedCurrentDir : TCmdStr;
   function GetCurrentDir:TCmdStr;
     begin
       if CachedCurrentDir='' then
         begin
           GetDir(0,CachedCurrentDir);
           CachedCurrentDir:=FixPath(CachedCurrentDir,false);
         end;
       result:=CachedCurrentDir;
     end;

   {Gives the relative path to the current directory,
    with a trailing dir separator. E. g. on unix ./ }
   function CurDirRelPath(systeminfo: tsysteminfo): TCmdStr;

   begin
     if systeminfo.system <> system_powerpc_macos then
       CurDirRelPath:= '.'+systeminfo.DirSep
     else
       CurDirRelPath:= ':'
   end;


   function path_absolute(const s : TCmdStr) : boolean;
   {
     is path s an absolute path?
   }
     begin
        result:=false;
{$if defined(unix)}
        if (length(s)>0) and (s[1] in AllowDirectorySeparators) then
          result:=true;
{$elseif defined(hasamiga)}
        (* An Amiga path is absolute, if it has a volume/device name in it (contains ":"),
           otherwise it's always a relative path, no matter if it starts with a directory
           separator or not. (KB) *)
        if (length(s)>0) and (Pos(':',s) <> 0) then
          result:=true;
{$elseif defined(macos)}
        if IsMacFullPath(s) then
          result:=true;
{$elseif defined(netware)}
        if (Pos (DriveSeparator, S) <> 0) or
                ((Length (S) > 0) and (S [1] in AllowDirectorySeparators)) then
          result:=true;
{$elseif defined(win32) or defined(win64) or defined(go32v2) or defined(os2) or defined(watcom)}
        if ((length(s)>0) and (s[1] in AllowDirectorySeparators)) or
(* The following check for non-empty AllowDriveSeparators assumes that all
   other platforms supporting drives and not handled as exceptions above
   should work with DOS-like paths, i.e. use absolute paths with one letter
   for drive followed by path separator *)
           ((length(s)>2) and (s[2] in AllowDriveSeparators) and (s[3] in AllowDirectorySeparators)) then
          result:=true;
{$else}
        if ((length(s)>0) and (s[1] in AllowDirectorySeparators)) or
(* The following check for non-empty AllowDriveSeparators assumes that all
   other platforms supporting drives and not handled as exceptions above
   should work with DOS-like paths, i.e. use absolute paths with one letter
   for drive followed by path separator *)
           ((AllowDriveSeparators <> []) and (length(s)>2) and (s[2] in AllowDriveSeparators) and (s[3] in AllowDirectorySeparators)) then
          result:=true;
{$endif unix}
     end;

    Function FileExists ( Const F : TCmdStr;allowcache:boolean) : Boolean;
      begin
{$ifdef usedircache}
        if allowcache then
          Result:=DirCache.FileExists(F)
        else
{$endif usedircache}
          Result:=SysUtils.FileExists(F);
        if do_checkverbosity(V_Tried) then
         begin
           if Result then
             do_comment(V_Tried,'Searching file '+F+'... found')
           else
             do_comment(V_Tried,'Searching file '+F+'... not found');
         end;
      end;


    function FileExistsNonCase(const path,fn:TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
      var
        fn2 : TCmdStr;
      begin
        result:=false;
        if tf_files_case_sensitive in source_info.flags then
          begin
            {
              Search order for case sensitive systems:
               1. NormalCase
               2. lowercase
               3. UPPERCASE
            }
            FoundFile:=path+fn;
            If FileExists(FoundFile,allowcache) then
             begin
               result:=true;
               exit;
             end;
            fn2:=Lower(fn);
            if fn2<>fn then
              begin
                FoundFile:=path+fn2;
                If FileExists(FoundFile,allowcache) then
                 begin
                   result:=true;
                   exit;
                 end;
              end;
            fn2:=Upper(fn);
            if fn2<>fn then
              begin
                FoundFile:=path+fn2;
                If FileExists(FoundFile,allowcache) then
                 begin
                   result:=true;
                   exit;
                 end;
              end;
          end
        else
          if tf_files_case_aware in source_info.flags then
            begin
              {
                Search order for case aware systems:
                 1. NormalCase
              }
{$ifdef usedircache}
              if allowcache then
                begin
                  result:=DirCache.FileExistsCaseAware(path,fn,fn2);
                  if result then
                    begin
                      FoundFile:=fn2;
                      exit;
                    end;
                end
              else
{$endif usedircache}
                begin
                  FoundFile:=path+fn;
                  If FileExists(FoundFile,allowcache) then
                    begin
                      { don't know the real name in this case }
                      result:=true;
                      exit;
                   end;
                end;
           end
        else
          begin
            { None case sensitive only lowercase }
            FoundFile:=path+Lower(fn);
            If FileExists(FoundFile,allowcache) then
             begin
               result:=true;
               exit;
             end;
          end;
        { Set foundfile to something useful }
        FoundFile:=fn;
      end;


    Function PathExists (const F : TCmdStr;allowcache:boolean) : Boolean;
      Var
        i: longint;
        hs : TCmdStr;
      begin
        if F = '' then
          begin
            result := true;
            exit;
          end;
        hs := ExpandFileName(F);
        I := Pos (DriveSeparator, hs);
        if (hs [Length (hs)] = DirectorySeparator) and
           (((I = 0) and (Length (hs) > 1)) or (I <> Length (hs) - 1)) then
          Delete (hs, Length (hs), 1);
{$ifdef usedircache}
        if allowcache then
          Result:=DirCache.DirectoryExists(hs)
        else
{$endif usedircache}
          Result:=SysUtils.DirectoryExists(hs);
      end;


    Function RemoveDir(d:TCmdStr):boolean;
      begin
        if d[length(d)]=source_info.DirSep then
         Delete(d,length(d),1);
        {$push}{$I-}
         rmdir(d);
        {$pop}
        RemoveDir:=(ioresult=0);
      end;


    Function FixPath(const s:TCmdStr;allowdot:boolean):TCmdStr;
      var
        i, L : longint;
        P: PChar;
      begin
        Result := s;
        { make result unique since we're going to change it via a pchar }
        uniquestring(result);
        L := Length(Result);
        if L=0 then
          exit;
        { Fix separator }
        P := @Result[1];
        for i:=0 to L-1 do
          begin
            if p^ in ['/','\'] then
              p^:=source_info.DirSep;
            inc(p);
          end;
        { Fix ending / }
        if (L>0) and (Result[L]<>source_info.DirSep) and
           (Result[L]<>DriveSeparator) then
          Result:=Result+source_info.DirSep;  { !still results in temp AnsiString }
        { Remove ./ }
        if (not allowdot) and ((Length(Result)=2) and (Result[1]='.') and (Result[2] = source_info.DirSep)) then
          begin
            Result:='';
            Exit;
          end;
        { return }
        if not ((tf_files_case_aware in source_info.flags) or
           (tf_files_case_sensitive in source_info.flags)) then
          Result := lower(Result);
      end;

  {Actually the version in macutils.pp could be used,
   but that would not work for crosscompiling, so this is a slightly modified
   version of it.}
  function TranslatePathToMac (const path: TCmdStr; mpw: Boolean): TCmdStr;

    function GetVolumeIdentifier: TCmdStr;

    begin
      GetVolumeIdentifier := '{Boot}'
      (*
      if mpw then
        GetVolumeIdentifier := '{Boot}'
      else
        GetVolumeIdentifier := macosBootVolumeName;
      *)
    end;

    var
      slashPos, oldpos, newpos, oldlen, maxpos: Longint;

  begin
    oldpos := 1;
    slashPos := Pos('/', path);
    TranslatePathToMac:='';
    if (slashPos <> 0) then   {its a unix path}
      begin
        if slashPos = 1 then
          begin      {its a full path}
            oldpos := 2;
            TranslatePathToMac := GetVolumeIdentifier;
          end
        else     {its a partial path}
          TranslatePathToMac := ':';
      end
    else
      begin
        slashPos := Pos('\', path);
        if (slashPos <> 0) then   {its a dos path}
          begin
            if slashPos = 1 then
              begin      {its a full path, without drive letter}
                oldpos := 2;
                TranslatePathToMac := GetVolumeIdentifier;
              end
            else if (Length(path) >= 2) and (path[2] = ':') then {its a full path, with drive letter}
              begin
                oldpos := 4;
                TranslatePathToMac := GetVolumeIdentifier;
              end
            else     {its a partial path}
              TranslatePathToMac := ':';
          end;
      end;

    if (slashPos <> 0) then   {its a unix or dos path}
      begin
        {Translate "/../" to "::" , "/./" to ":" and "/" to ":" }
        newpos := Length(TranslatePathToMac);
        oldlen := Length(path);
        SetLength(TranslatePathToMac, newpos + oldlen);  {It will be no longer than what is already}
                                                                        {prepended plus length of path.}
        maxpos := Length(TranslatePathToMac);          {Get real maxpos, can be short if String is ShortString}

        {There is never a slash in the beginning, because either it was an absolute path, and then the}
        {drive and slash was removed, or it was a relative path without a preceding slash.}
        while oldpos <= oldlen do
          begin
            {Check if special dirs, ./ or ../ }
            if path[oldPos] = '.' then
              if (oldpos + 1 <= oldlen) and (path[oldPos + 1] = '.') then
                begin
                  if (oldpos + 2 > oldlen) or (path[oldPos + 2] in ['/', '\']) then
                    begin
                      {It is "../" or ".."  translates to ":" }
                      if newPos = maxPos then
                        begin {Shouldn't actually happen, but..}
                          Exit('');
                        end;
                      newPos := newPos + 1;
                      TranslatePathToMac[newPos] := ':';
                      oldPos := oldPos + 3;
                      continue;  {Start over again}
                    end;
                end
              else if (oldpos + 1 > oldlen) or (path[oldPos + 1] in ['/', '\']) then
                begin
                  {It is "./" or "."  ignor it }
                  oldPos := oldPos + 2;
                  continue;  {Start over again}
                end;

            {Collect file or dir name}
            while (oldpos <= oldlen) and not (path[oldPos] in ['/', '\']) do
              begin
                if newPos = maxPos then
                  begin {Shouldn't actually happen, but..}
                    Exit('');
                  end;
                newPos := newPos + 1;
                TranslatePathToMac[newPos] := path[oldPos];
                oldPos := oldPos + 1;
              end;

            {When we come here there is either a slash or we are at the end.}
            if (oldpos <= oldlen) then
              begin
                if newPos = maxPos then
                  begin {Shouldn't actually happen, but..}
                    Exit('');
                  end;
                newPos := newPos + 1;
                TranslatePathToMac[newPos] := ':';
                oldPos := oldPos + 1;
              end;
          end;

        SetLength(TranslatePathToMac, newpos);
      end
    else if (path = '.') then
      TranslatePathToMac := ':'
    else if (path = '..') then
      TranslatePathToMac := '::'
    else
      TranslatePathToMac := path;  {its a mac path}
  end;


   function FixFileName(const s:TCmdStr):TCmdStr;
     var
       i      : longint;
     begin
       if source_info.system = system_powerpc_MACOS then
         FixFileName:= TranslatePathToMac(s, true)
       else
        if (tf_files_case_aware in source_info.flags) or
           (tf_files_case_sensitive in source_info.flags) then
        begin
          setlength(FixFileName,length(s));
          for i:=1 to length(s) do
           begin
             case s[i] of
               '/','\' :
                 FixFileName[i]:=source_info.dirsep;
               else
                 FixFileName[i]:=s[i];
             end;
           end;
        end
       else
        begin
          setlength(FixFileName,length(s));
          for i:=1 to length(s) do
           begin
             case s[i] of
               '/','\' :
                  FixFileName[i]:=source_info.dirsep;
               'A'..'Z' :
                  FixFileName[i]:=char(byte(s[i])+32);
                else
                  FixFileName[i]:=s[i];
             end;
           end;
        end;
     end;


    Function TargetFixPath(s:TCmdStr;allowdot:boolean):TCmdStr;
      var
        i : longint;
      begin
        { Fix separator }
        for i:=1 to length(s) do
         if s[i] in ['/','\'] then
          s[i]:=target_info.DirSep;
        { Fix ending / }
        if (length(s)>0) and (s[length(s)]<>target_info.DirSep) and
           (s[length(s)]<>':') then
         s:=s+target_info.DirSep;
        { Remove ./ }
        if (not allowdot) and (s='.'+target_info.DirSep) then
         s:='';
        { return }
        if (tf_files_case_aware in target_info.flags) or
           (tf_files_case_sensitive in target_info.flags) then
         TargetFixPath:=s
        else
         TargetFixPath:=Lower(s);
      end;


   function TargetFixFileName(const s:TCmdStr):TCmdStr;
     var
       i : longint;
     begin
       if target_info.system = system_powerpc_MACOS then
         TargetFixFileName:= TranslatePathToMac(s, true)
       else
        if (tf_files_case_aware in target_info.flags) or
           (tf_files_case_sensitive in target_info.flags) then
         begin
           setlength(TargetFixFileName,length(s));
           for i:=1 to length(s) do
           begin
             case s[i] of
               '/','\' :
                 TargetFixFileName[i]:=target_info.dirsep;
               else
                 TargetFixFileName[i]:=s[i];
             end;
           end;
         end
       else
         begin
           setlength(TargetFixFileName,length(s));
           for i:=1 to length(s) do
           begin
             case s[i] of
               '/','\' :
                  TargetFixFileName[i]:=target_info.dirsep;
               'A'..'Z' :
                  TargetFixFileName[i]:=char(byte(s[i])+32);
                else
                  TargetFixFileName[i]:=s[i];
             end;
           end;
         end;
     end;


   procedure SplitBinCmd(const s:TCmdStr;var bstr:TCmdStr;var cstr:TCmdStr);
     var
       i : longint;
     begin
       i:=pos(' ',s);
       if i>0 then
        begin
          bstr:=Copy(s,1,i-1);
          cstr:=Copy(s,i+1,length(s)-i);
        end
       else
        begin
          bstr:=s;
          cstr:='';
        end;
     end;


    procedure TSearchPathList.AddPath(s:TCmdStr;addfirst:boolean);
      begin
        AddLibraryPath('',s,AddFirst);
      end;


   procedure TSearchPathList.AddLibraryPath(const sysroot: TCmdStr; s:TCmdStr;addfirst:boolean);
     var
       staridx,
       i,j      : longint;
       prefix,
       suffix,
       CurrentDir,
       currPath : TCmdStr;
       subdirfound : boolean;
{$ifdef usedircache}
       dir      : TCachedSearchRec;
{$else usedircache}
       dir      : TSearchRec;
{$endif usedircache}
       hp       : TCmdStrListItem;

       procedure WarnNonExistingPath(const path : TCmdStr);
       begin
         if do_checkverbosity(V_Tried) then
           do_comment(V_Tried,'Path "'+path+'" not found');
       end;

       procedure AddCurrPath;
       begin
         if addfirst then
          begin
            Remove(currPath);
            Insert(currPath);
          end
         else
          begin
            { Check if already in path, then we don't add it }
            hp:=Find(currPath);
            if not assigned(hp) then
             Concat(currPath);
          end;
       end;

     begin
       if s='' then
        exit;
     { Support default macro's }
       DefaultReplacements(s);
{$warnings off}
       if PathSeparator <> ';' then
        for i:=1 to length(s) do
         if s[i]=PathSeparator then
          s[i]:=';';
{$warnings on}
     { get current dir }
       CurrentDir:=GetCurrentDir;
       repeat
         { get currpath }
         if addfirst then
          begin
            j:=length(s);
            while (j>0) and (s[j]<>';') do
             dec(j);
            currPath:= TrimSpace(Copy(s,j+1,length(s)-j));
            if j=0 then
             s:=''
            else
             System.Delete(s,j,length(s)-j+1);
          end
         else
          begin
            j:=Pos(';',s);
            if j=0 then
             j:=length(s)+1;
            currPath:= TrimSpace(Copy(s,1,j-1));
            System.Delete(s,1,j);
          end;

         { fix pathname }
         DePascalQuote(currPath);
         { GNU LD convention: if library search path starts with '=', it's relative to the
           sysroot; otherwise, interpret it as a regular path }
         if (length(currPath) >0) and (currPath[1]='=') then
           currPath:=sysroot+FixPath(copy(currPath,2,length(currPath)-1),false);
         if currPath='' then
           currPath:= CurDirRelPath(source_info)
         else
          begin
            currPath:=FixPath(ExpandFileName(currpath),false);
            if (CurrentDir<>'') and (Copy(currPath,1,length(CurrentDir))=CurrentDir) then
             begin
{$ifdef hasamiga}
               currPath:= CurrentDir+Copy(currPath,length(CurrentDir)+1,length(currPath));
{$else}
               currPath:= CurDirRelPath(source_info)+Copy(currPath,length(CurrentDir)+1,length(currPath));
{$endif}
             end;
          end;
         { wildcard adding ? }
         staridx:=pos('*',currpath);
         if staridx>0 then
          begin
            prefix:=ExtractFilePath(Copy(currpath,1,staridx));
            suffix:=Copy(currpath,staridx+1,length(currpath));
            subdirfound:=false;
{$ifdef usedircache}
            if DirCache.FindFirst(Prefix+AllFilesMask,dir) then
              begin
                repeat
                  if (dir.attr and faDirectory)<>0 then
                    begin
                      subdirfound:=true;
                      currpath:=prefix+dir.name+suffix;
                      if (suffix='') or PathExists(currpath,true) then
                        begin
                          hp:=Find(currPath);
                          if not assigned(hp) then
                            AddCurrPath;
                        end;
                    end;
                until not DirCache.FindNext(dir);
              end;
            DirCache.FindClose(dir);
{$else usedircache}
            if findfirst(prefix+AllFilesMask,faDirectory,dir) = 0 then
              begin
                repeat
                  if (dir.name<>'.') and
                      (dir.name<>'..') and
                      ((dir.attr and faDirectory)<>0) then
                    begin
                      subdirfound:=true;
                      currpath:=prefix+dir.name+suffix;
                      if (suffix='') or PathExists(currpath,false) then
                        begin
                          hp:=Find(currPath);
                          if not assigned(hp) then
                            AddCurrPath;
                        end;
                    end;
                until findnext(dir) <> 0;
                FindClose(dir);
              end;
{$endif usedircache}
            if not subdirfound then
              WarnNonExistingPath(currpath);
          end
         else
          begin
            if PathExists(currpath,true) then
             AddCurrPath
            else
             WarnNonExistingPath(currpath);
          end;
       until (s='');
     end;


   procedure TSearchPathList.AddList(list:TSearchPathList;addfirst:boolean);
     var
       s : TCmdStr;
       hl : TSearchPathList;
       hp,hp2 : TCmdStrListItem;
     begin
       if list.empty then
        exit;
       { create temp and reverse the list }
       if addfirst then
        begin
          hl:=TSearchPathList.Create;
          hp:=TCmdStrListItem(list.first);
          while assigned(hp) do
           begin
             hl.insert(hp.Str);
             hp:=TCmdStrListItem(hp.next);
           end;
          while not hl.empty do
           begin
             s:=hl.GetFirst;
             Remove(s);
             Insert(s);
           end;
          hl.Free;
        end
       else
        begin
          hp:=TCmdStrListItem(list.first);
          while assigned(hp) do
           begin
             hp2:=Find(hp.Str);
             { Check if already in path, then we don't add it }
             if not assigned(hp2) then
              Concat(hp.Str);
             hp:=TCmdStrListItem(hp.next);
           end;
        end;
     end;


   function TSearchPathList.FindFile(const f :TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
     Var
       p : TCmdStrListItem;
     begin
       FindFile:=false;
       p:=TCmdStrListItem(first);
       while assigned(p) do
        begin
          result:=FileExistsNonCase(p.Str,f,allowcache,FoundFile);
          if result then
            exit;
          p:=TCmdStrListItem(p.next);
        end;
       { Return original filename if not found }
       FoundFile:=f;
     end;


   function FindFile(const f : TCmdStr; const path : TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
     Var
       StartPos, EndPos, L: LongInt;
     begin
       Result:=False;

       if (path_absolute(f)) then
         begin
           Result:=FileExistsNonCase('',f, allowcache, foundfile);
           if Result then
             Exit;
         end;

       StartPos := 1;
       L := Length(Path);
       repeat
         EndPos := StartPos;
         while (EndPos <= L) and ((Path[EndPos] <> PathSeparator) and (Path[EndPos] <> ';')) do
           Inc(EndPos);
         Result := FileExistsNonCase(FixPath(Copy(Path, StartPos, EndPos-StartPos), False), f, allowcache, FoundFile);
         if Result then
           Exit;
         StartPos := EndPos + 1;
       until StartPos > L;
       FoundFile:=f;
     end;

{
   function FindFilePchar(const f : TCmdStr;path : pchar;allowcache:boolean;var foundfile:TCmdStr):boolean;
      Var
        singlepathstring : TCmdStr;
        startpc,pc : pchar;
     begin
       FindFilePchar:=false;
       if Assigned (Path) then
        begin
          pc:=path;
          repeat
             startpc:=pc;
             while (pc^<>PathSeparator) and (pc^<>';') and (pc^<>#0) do
              inc(pc);
             SetLength(singlepathstring, pc-startpc);
             move(startpc^,singlepathstring[1],pc-startpc);
             singlepathstring:=FixPath(ExpandFileName(singlepathstring),false);
             result:=FileExistsNonCase(singlepathstring,f,allowcache,FoundFile);
             if result then
               exit;
             if (pc^=#0) then
               break;
             inc(pc);
          until false;
        end;
       foundfile:=f;
     end;
}

  function  FindFileInExeLocations(const bin:TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
    var
      Path : TCmdStr;
      found : boolean;
    begin
       found:=FindFile(FixFileName(bin),exepath,allowcache,foundfile);
      if not found then
       begin
{$ifdef macos}
         Path:=GetEnvironmentVariable('Commands');
{$else}
         Path:=GetEnvironmentVariable('PATH');
{$endif}
         found:=FindFile(FixFileName(bin),Path,allowcache,foundfile);
       end;
      FindFileInExeLocations:=found;
    end;


   function  FindExe(const bin:TCmdStr;allowcache:boolean;var foundfile:TCmdStr):boolean;
     var
       b : TCmdStr;
     begin
       { change extension only on platforms that use an exe extension, otherwise on OpenBSD
         'ld.bfd' gets converted to 'ld' }
       if source_info.exeext<>'' then
         b:=ChangeFileExt(bin,source_info.exeext)
       else
         b:=bin;
       FindExe:=FindFileInExeLocations(b,allowcache,foundfile);
     end;


    function GetShortName(const n:TCmdStr):TCmdStr;
{$ifdef win32}
      var
        hs,hs2 : TCmdStr;
        i : longint;
{$endif}
{$if defined(go32v2) or defined(watcom)}
      var
        hs : shortstring;
{$endif}
      begin
        GetShortName:=n;
{$ifdef win32}
        hs:=n+#0;
        hs2:='';
        { may become longer in case of e.g. ".a" -> "a~1" or so }
        setlength(hs2,length(hs)*2);
        i:=Windows.GetShortPathName(@hs[1],@hs2[1],length(hs)*2);
        if (i>0) and (i<=length(hs)*2) then
          begin
            setlength(hs2,strlen(@hs2[1]));
            GetShortName:=hs2;
          end;
{$endif}
{$if defined(go32v2) or defined(watcom)}
        hs:=n;
        if Dos.GetShortName(hs) then
         GetShortName:=hs;
{$endif}
      end;


    function maybequoted(const s:string):string;
    const
      FORBIDDEN_CHARS_DOS = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
                         '{', '}', '''', '`', '~'];
      FORBIDDEN_CHARS_OTHER = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
                         '{', '}', '''', ':', '\', '`', '~'];
    var
      forbidden_chars: set of char;
      i  : integer;
      quote_script: tscripttype;
      quote_char: ansichar;
      quoted : boolean;
    begin
      if not(cs_link_on_target in current_settings.globalswitches) then
        quote_script:=source_info.script
      else
        quote_script:=target_info.script;
      if quote_script=script_dos then
        forbidden_chars:=FORBIDDEN_CHARS_DOS
      else
        begin
          forbidden_chars:=FORBIDDEN_CHARS_OTHER;
          if quote_script=script_unix then
            include(forbidden_chars,'"');
        end;
      if quote_script=script_unix then
        quote_char:=''''
      else
        quote_char:='"';

      quoted:=false;
      result:=quote_char;
      for i:=1 to length(s) do
       begin
         if s[i]=quote_char then
           begin
             quoted:=true;
             result:=result+'\'+quote_char;
           end
         else case s[i] of
           '\':
             begin
               if quote_script=script_unix then
                 begin
                   result:=result+'\\';
                   quoted:=true
                 end
               else
                 result:=result+'\';
             end;
           ' ',
           #128..#255 :
             begin
               quoted:=true;
               result:=result+s[i];
             end;
           else begin
             if s[i] in forbidden_chars then
               quoted:=True;
             result:=result+s[i];
           end;
         end;
       end;
      if quoted then
        result:=result+quote_char
      else
        result:=s;
    end;


    function maybequoted_for_script(const s:ansistring; quote_script: tscripttype):ansistring;
      const
        FORBIDDEN_CHARS_DOS = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
                           '{', '}', '''', '`', '~'];
        FORBIDDEN_CHARS_OTHER = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
                           '{', '}', '''', ':', '\', '`', '~'];
      var
        forbidden_chars: set of char;
        i  : integer;
        quote_char: ansichar;
        quoted : boolean;
      begin
        if quote_script=script_dos then
          forbidden_chars:=FORBIDDEN_CHARS_DOS
        else
          begin
            forbidden_chars:=FORBIDDEN_CHARS_OTHER;
            if quote_script=script_unix then
              include(forbidden_chars,'"');
          end;
        if quote_script=script_unix then
          quote_char:=''''
        else
          quote_char:='"';

        quoted:=false;
        result:=quote_char;
        for i:=1 to length(s) do
         begin
           if s[i]=quote_char then
             begin
               quoted:=true;
               result:=result+'\'+quote_char;
             end
           else case s[i] of
             '\':
               begin
                 if quote_script=script_unix then
                   begin
                     result:=result+'\\';
                     quoted:=true
                   end
                 else
                   result:=result+'\';
               end;
             ' ',
             #128..#255 :
               begin
                 quoted:=true;
                 result:=result+s[i];
               end;
             else begin
               if s[i] in forbidden_chars then
                 quoted:=True;
               result:=result+s[i];
             end;
           end;
         end;
        if quoted then
          result:=result+quote_char
        else
          result:=s;
      end;


    function maybequoted(const s:ansistring):ansistring;
      var
        quote_script: tscripttype;
      begin
        if not(cs_link_on_target in current_settings.globalswitches) then
          quote_script:=source_info.script
        else
          quote_script:=target_info.script;
        result:=maybequoted_for_script(s,quote_script);
      end;


    { requotes a string that was quoted for Unix for passing to ExecuteProcess,
      because it only supports Windows-style quoting; this routine assumes that
      everything that has to be quoted for Windows, was also quoted (but
      differently for Unix) -- which is the case }
    function UnixRequoteWithDoubleQuotes(const QuotedStr: TCmdStr): TCmdStr;
      var
        i: longint;
        temp: TCmdStr;
        inquotes: boolean;
      begin
        if QuotedStr='' then
          begin
            result:='';
            exit;
          end;
        inquotes:=false;
        result:='';
        i:=1;
        temp:='';
        while i<=length(QuotedStr) do
          begin
            case QuotedStr[i] of
              '''':
                begin
                  if not(inquotes) then
                    begin
                      inquotes:=true;
                      temp:=''
                    end
                  else
                    begin
                      { requote for Windows }
                      result:=result+maybequoted_for_script(temp,script_dos);
                      inquotes:=false;
                    end;
                end;
              '\':
                begin
                  if inquotes then
                    temp:=temp+QuotedStr[i+1]
                  else
                    result:=result+QuotedStr[i+1];
                  inc(i);
                end;
              else
                begin
                  if inquotes then
                    temp:=temp+QuotedStr[i]
                  else
                    result:=result+QuotedStr[i];
                end;
            end;
            inc(i);
          end;
      end;


    function RequotedExecuteProcess(const Path: AnsiString; const ComLine: AnsiString; Flags: TExecuteFlags): Longint;
      var
        quote_script: tscripttype;
      begin

        if do_checkverbosity(V_Executable) then
          do_comment(V_Executable,'Executing "'+Path+'" with command line "'+
            ComLine+'"');
        if (cs_link_on_target in current_settings.globalswitches) then
          quote_script:=target_info.script
        else
          quote_script:=source_info.script;
        if quote_script=script_unix then
          result:=sysutils.ExecuteProcess(Path,UnixRequoteWithDoubleQuotes(ComLine),Flags)
        else
          result:=sysutils.ExecuteProcess(Path,ComLine,Flags)
      end;


    function RequotedExecuteProcess(const Path: AnsiString; const ComLine: array of AnsiString; Flags: TExecuteFlags): Longint;
      var
        i : longint;
        st : string;
      begin
        if do_checkverbosity(V_Executable) then
          begin
            if high(ComLine)=0 then
              st:=''
            else
              st:=ComLine[1];
            for i:=2 to high(ComLine) do
              st:=st+' '+ComLine[i];
            do_comment(V_Executable,'Executing "'+Path+'" with command line "'+
              st+'"');
          end;
        result:=sysutils.ExecuteProcess(Path,ComLine,Flags);
      end;


    function Shell(const command:ansistring): longint;
      { This is already defined in the linux.ppu for linux, need for the *
        expansion under linux }
{$ifdef hasunix}
      begin
        do_comment(V_Executable,'Executing "'+Command+'" with fpSystem call');
        result := Unix.fpsystem(command);
      end;
{$else hasunix}
  {$ifdef hasamiga}
      begin
        do_comment(V_Executable,'Executing "'+Command+'" using RequotedExecuteProcess');
        result := RequotedExecuteProcess('',command);
      end;
  {$else hasamiga}
      var
        comspec : string;
      begin
        comspec:=GetEnvironmentVariable('COMSPEC');
        do_comment(V_Executable,'Executing "'+Command+'" using comspec "'
            +ComSpec+'"');
        result := RequotedExecuteProcess(comspec,' /C '+command);
      end;
   {$endif hasamiga}
{$endif hasunix}



{****************************************************************************
                           Init / Done
****************************************************************************}

    procedure InitFileUtils;
      begin
        DirCache:=TDirectoryCache.Create;
      end;


    procedure DoneFileUtils;
      begin
        DirCache.Free;
      end;

end.
