{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1993-2015 by Florian Klaempfl
    member of the Free Pascal development team

    This is the install program for the DOS and OS/2 versions of Free Pascal

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program install;

{ $DEFINE DLL}       (* TH - if defined, UNZIP32.DLL library is used to unpack. *)
{ $DEFINE DOSSTUB}   (* TH - should _not_ be defined unless creating a bound DOS and OS/2 installer!!! *)
(* Defining DOSSTUB causes adding a small piece of code    *)
(* for starting the OS/2 part from the DOS part of a bound *)
(* application if running in OS/2 VDM (DOS) window. Used   *)
(* only if compiling with TP/BP (see conditionals below).  *)

{$IFDEF OS2}
 {$DEFINE DLL}
{$ENDIF DLL}

{$IFDEF VER60}
 {$DEFINE TP}
{$ENDIF}

{$IFDEF VER70}
 {$DEFINE TP}
{$ENDIF}

{$IFNDEF TP}
 {$UNDEF DOSSTUB}
{$ELSE}
 {$IFDEF OS2}
  {$UNDEF DOSSTUB}
 {$ENDIF}
{$ENDIF}

{$IFDEF DPMI}
 {$UNDEF DOSSTUB}
{$ENDIF}

{$ifdef go32v2}
{$define MAYBE_LFN}
{$endif}

{$ifdef debug}
{$ifdef win32}
{$define MAYBE_LFN}
{$endif win32}
{$endif debug}

{$ifdef TP}
{$define MAYBE_LFN}
{$endif}



  uses
{$IFDEF OS2}
 {$IFDEF FPC}
     DosCalls,
 {$ELSE FPC}
  {$IFDEF VirtualPascal}
     OS2Base,
  {$ELSE VirtualPascal}
     BseDos,
  {$ENDIF VirtualPascal}
 {$ENDIF FPC}
{$ENDIF OS2}
{$IFDEF GO32V2}
     emu387,
{$ENDIF}
{$ifdef HEAPTRC}
     heaptrc,
{$endif HEAPTRC}
     strings,dos,objects,drivers,
{$IFNDEF FVISION}
     commands,
     HelpCtx,
{$ENDIF}
     unzip51g,ziptypes,
{$IFDEF DLL}
     unzipdll,
{$ENDIF}
     app,dialogs,views,menus,msgbox,colortxt,tabs,scroll,
     WHTMLScn,insthelp;

  const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     installerversion='3.1.1';
     installercopyright='Copyright (c) 1993-2015 Florian Klaempfl';


     maxpacks=20;
     maxpackages=32;
=======
     installerversion='2.5.1';
     installercopyright='Copyright (c) 1993-2011 Florian Klaempfl';


     maxpacks=20;
     maxpackages=40;
>>>>>>> graemeg/cpstrnew
=======
     installerversion='2.5.1';
     installercopyright='Copyright (c) 1993-2011 Florian Klaempfl';


     maxpacks=20;
     maxpackages=40;
>>>>>>> graemeg/cpstrnew
=======
     installerversion='2.5.1';
     installercopyright='Copyright (c) 1993-2011 Florian Klaempfl';


     maxpacks=20;
     maxpackages=40;
>>>>>>> graemeg/cpstrnew
=======
     installerversion='2.5.1';
     installercopyright='Copyright (c) 1993-2011 Florian Klaempfl';


     maxpacks=20;
     maxpackages=40;
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     installerversion='2.2.5';
     installercopyright='Copyright (c) 1993-2009 Florian Klaempfl';


     maxpacks=30;
<<<<<<< HEAD
     maxpackages=28;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     maxpackages=29;
>>>>>>> origin/fixes_2.4
     maxdefcfgs=1024;

     HTMLIndexExt = '.htx';
     CfgExt = '.dat';

     MaxStatusPos = 4;
     StatusChars: string [MaxStatusPos] = '/-\|';
     StatusPos: byte = 1;
     { this variable is set to true if an ide is installed }
     haside : boolean = false;
     hashtmlhelp : boolean = false;

{$ifdef Unix}
     DirSep='/';
{$else}
     DirSep='\';
{$endif}

  type
     tpackage=record
       name      : string[60];
       zip       : string[40];  { default zipname }
       zipshort  : string[12];  { 8.3 zipname }
       diskspace : int64;     { diskspace required }
     end;

     tpack=record
       name     : string[12];
       binsub   : string[40];
       ppc386   : string[20];
       targetname : string[40];
       defidecfgfile,
       defideinifile,
       defcfgfile,
       setpathfile : string[12];
       include  : boolean;
       { filechk  : string[40]; Obsolete }
       packages : longint;
       package  : array[1..maxpackages] of tpackage;
     end;

     tcfgarray = array[1..maxdefcfgs] of pstring;

     cfgrec=record
       title    : string[80];
       version  : string[20];
       helpidx,
       docsub,
       basepath : DirStr;
       packs    : word;
       pack     : array[1..maxpacks] of tpack;
       defideinis,
       defidecfgs,
       defcfgs,
       defsetpaths : longint;
       defideini,
       defidecfg,
       defcfg,
       defsetpath : tcfgarray;
     end;

     datarec=record
       basepath : DirStr;
       cfgval   : word;
       packmask : array[1..maxpacks] of sw_word;
     end;

     punzipdialog=^tunzipdialog;
     tunzipdialog=object(tdialog)
        filetext : pstatictext;
        extractfiletext : pstatictext;
        currentfile : string;
        constructor Init(var Bounds: TRect; ATitle: TTitleStr);
        procedure do_unzip(s,topath:string);
     end;

     penddialog = ^tenddialog;
     tenddialog = object(tdialog)
        constructor init;
     end;

     pinstalldialog = ^tinstalldialog;
     tinstalldialog = object(tdialog)
        constructor init;
        procedure handleevent(var event : tevent);virtual;
     end;

     PFPHTMLFileLinkScanner = ^TFPHTMLFileLinkScanner;
     TFPHTMLFileLinkScanner = object(THTMLFileLinkScanner)
        function    CheckURL(const URL: string): boolean; virtual;
        function    CheckText(const Text: string): boolean; virtual;
        procedure   ProcessDoc(Doc: PHTMLLinkScanFile); virtual;
     end;

     phtmlindexdialog = ^thtmlindexdialog;
     thtmlindexdialog = object(tdialog)
       text : pstatictext;
       constructor init(var Bounds: TRect; ATitle: TTitleStr);
     end;

     tapp = object(tapplication)
         procedure initmenubar;virtual;
         procedure initstatusline;virtual;
         procedure handleevent(var event : tevent);virtual;
         procedure do_installdialog;
         procedure readcfg(const fn:string);
         procedure checkavailpack;
     end;

     PSpecialInputLine= ^TSpecialInputLine;
     TSpecialInputLine = object (TInputLine)
       procedure GetData(var Rec); virtual;
     end;

{$IFDEF DOSSTUB}
  PByte = ^byte;
  PRunBlock = ^TRunBlock;
  TRunBlock = record
    Length: word;
    Dependent: word;
    Background: word;
    TraceLevel: word;
    PrgTitle: PChar;
    PrgName: PChar;
    Args: PChar;
    TermQ: longint;
    Environment: pointer;
    Inheritance: word;
    SesType: word;
    Icon: pointer;
    PgmHandle: longint;
    PgmControl: word;
    Column: word;
    Row: word;
    Width: word;
    Height: word;
  end;
{$ENDIF}

  var
     installapp  : tapp;
     startpath   : string;
     successfull : boolean;
     cfg         : cfgrec;
     data        : datarec;
     CfgName: NameStr;
     DStr: DirStr;
     EStr: ExtStr;
     UnzDlg      : punzipdialog;
     log         : text;
     createlog   : boolean;

{$IFNDEF DLL}
  const
     UnzipErr: longint = 0;
{$ENDIF}
{$ifdef MAYBE_LFN}
  const
    locallfnsupport : boolean = false;
{$endif MAYBE_LFN}


{*****************************************************************************
                                  Helpers
*****************************************************************************}

  procedure errorhalt;
    begin
      installapp.done;
      if CreateLog then
        begin
          WriteLn (Log, 'Installation hasn''t been completed.');
          Close (Log);
        end;
      halt(1);
    end;


  procedure WriteLog (const S: string);
    begin
      if CreateLog then
        begin
          WriteLn (Log, S);
          Flush (Log);
        end;
    end;

  function packagemask(i:longint):longint;
    begin
      packagemask:=1 shl (i-1);
    end;


  function upper(const s : string):string;
    var
       i : integer;
    begin
       for i:=1 to length(s) do
         if s[i] in ['a'..'z'] then
          upper[i]:=chr(ord(s[i])-32)
         else
          upper[i]:=s[i];
       upper[0]:=s[0];
    end;


  procedure Replace(var s:string;const s1,s2:string);
    var
       i  : longint;
    begin
      repeat
        i:=pos(s1,s);
        if i>0 then
         begin
           Delete(s,i,length(s1));
           Insert(s2,s,i);
         end;
      until i=0;
    end;


  function DotStr(l:longint):string;
    var
      TmpStr : string[32];
      i : longint;
    begin
      Str(l,TmpStr);
      i:=Length(TmpStr);
      while (i>3) do
       begin
         i:=i-3;
         if TmpStr[i]<>'-' then
          Insert('.',TmpStr,i+1);
       end;
      DotStr:=TmpStr;
    end;


  function file_exists(const f : string;const path : string) : boolean;
    begin
       file_exists:=fsearch(f,path)<>'';
    end;


  function createdir(s:string):boolean;
    var
      s1,start : string;
      err : boolean;
      i : longint;
    begin
       err:=false;
       {$I-}
       getdir(0,start);
{$ifndef Unix}
       if (s[2]=':') and (s[3]=DirSep) then
        begin
          chdir(Copy(s,1,3));
          Delete(S,1,3);
        end;
{$endif}
       repeat
         i:=Pos(DirSep,s);
         if i=0 then
          i:=255;
         s1:=Copy(s,1,i-1);
         Delete(s,1,i);
         ChDir(s1);
         if ioresult<>0 then
          begin
            mkdir(s1);
            chdir(s1);
            if ioresult<>0 then
             begin
               err:=true;
               break;
             end;
          end;
       until s='';
       chdir(start);
       {$I+}
       createdir:=err;
    end;


  function DiskSpaceN(const zipfile : string) : longint;
    var
      compressed,uncompressed : longint;
      s : string;
    begin
      s:=zipfile+#0;
      if not (IsZip (@S [1])) then
       DiskSpaceN := -1
      else
       begin
         Uncompressed:=UnzipSize(@s[1],compressed);
         DiskSpaceN:=uncompressed shr 10;
       end;
    end;


  function diskspacestr(uncompressed : longint) : string;
    begin
      if Uncompressed = -1 then
       DiskSpacestr := ' [INVALID]'
      else
       diskspacestr:=' ('+DotStr(uncompressed)+' KB)';
    end;


  function createinstalldir(s : string) : boolean;
    var
      err : boolean;
      dir : searchrec;
      params : array[0..0] of pointer;
    begin
       if s[length(s)]=DirSep then
        dec(s[0]);
       FindFirst(s,AnyFile,dir);
       if doserror=0 then
         begin
            if Dir.Attr and Directory = 0 then
              begin
                messagebox('A file with the name chosen as the installation '+
                'directory exists already. Cannot create this directory!',nil,
                mferror+mfokbutton);
                createinstalldir:=false;
              end else
                createinstalldir:=messagebox('The installation directory exists already. '+
                'Do you want to continue ?',nil,
                mferror+mfyesbutton+mfnobutton)=cmYes;
            exit;
         end;
       err:=Createdir(s);
       if err then
         begin
            params[0]:=@s;
            messagebox('The installation directory %s couldn''t be created',
              @params,mferror+mfokbutton);
            createinstalldir:=false;
            exit;
         end;
{$ifndef TP}
 {$IFNDEF OS2}
       FindClose (dir);
 {$ENDIF}
{$endif}
       createinstalldir:=true;
    end;


  function GetProgDir: DirStr;
    var
      D: DirStr;
      N: NameStr;
      E: ExtStr;
    begin
       FSplit (FExpand (ParamStr (0)), D, N, E);
       if (D [0] <> #0) and (D [byte (D [0])] = '\') then Dec (D [0]);
       GetProgDir := D;
    end;

  function GetZipErrorInfo(error : longint) : string;
  var
    ErrorStr : string;
  begin
    case error of
      unzip_CRCErr         : GetZipErrorInfo:='CRC error';
      unzip_WriteErr       : GetZipErrorInfo:='Write error';
      unzip_ReadErr        : GetZipErrorInfo:='Read error';
      unzip_ZipFileErr     : GetZipErrorInfo:='ZipFile erroe';
      unzip_UserAbort      : GetZipErrorInfo:='User abort';
      unzip_NotSupported   : GetZipErrorInfo:='Not supported';
      unzip_Encrypted      : GetZipErrorInfo:='File is encrypted';
      unzip_InUse          : GetZipErrorInfo:='Fie is in use';
      unzip_InternalError  : GetZipErrorInfo:='Internal error';    {Error in zip format}
      unzip_NoMoreItems    : GetZipErrorInfo:='No more items';
      unzip_FileError      : GetZipErrorInfo:='File error';   {Error Accessing file}
      unzip_NotZipfile     : GetZipErrorInfo:='Not a zipfile';   {not a zip file}
      unzip_SeriousError   : GetZipErrorInfo:='Serious error';  {serious error}
      unzip_MissingParameter : GetZipErrorInfo:='Missing parameter'; {missing parameter}
    else
      begin
        Str(Error,ErrorStr);
        GetZipErrorInfo:='Unknown error '+errorstr;
      end;
    end;
  end;

{*****************************************************************************
                          HTML-Index Generation
*****************************************************************************}
  var
     indexdlg : phtmlindexdialog;

  constructor thtmlindexdialog.Init(var Bounds: TRect; ATitle: TTitleStr);
    var
      r : trect;
    begin
      inherited init(bounds,atitle);
      Options:=Options or ofCentered;
      R.Assign (4, 2,bounds.B.X-Bounds.A.X-2, 4);
      text:=new(pstatictext,init(r,'Please wait ...'));
      insert(text);
    end;

  procedure TFPHTMLFileLinkScanner.ProcessDoc(Doc: PHTMLLinkScanFile);

    var
       oldtext : pstring;
    begin
       oldtext:=indexdlg^.text^.text;
       indexdlg^.text^.text:=newstr('Processing '+Doc^.GetDocumentURL);
       indexdlg^.text^.drawview;
       inherited ProcessDoc(Doc);
       disposestr(indexdlg^.text^.text);
       indexdlg^.text^.text:=oldtext;
       indexdlg^.text^.drawview;
    end;

  function TFPHTMLFileLinkScanner.CheckURL(const URL: string): boolean;
  var OK: boolean;
  const HTTPPrefix = 'http:';
        FTPPrefix  = 'ftp:';
  begin
    OK:=inherited CheckURL(URL);
    if OK then OK:=DirAndNameOf(URL)<>'';
    if OK then OK:=CompareText(copy(ExtOf(URL),1,4),'.HTM')=0;
    if OK then OK:=CompareText(copy(URL,1,length(HTTPPrefix)),HTTPPrefix)<>0;
    if OK then OK:=CompareText(copy(URL,1,length(FTPPrefix)),FTPPrefix)<>0;
    CheckURL:=OK;
  end;

  function TFPHTMLFileLinkScanner.CheckText(const Text: string): boolean;
  var OK: boolean;
      S: string;
  begin
    S:=Trim(Text);
    OK:=(S<>'') and (copy(S,1,1)<>'[');
    CheckText:=OK;
  end;

  procedure writehlpindex(filename : string);

    var
       LS : PFPHTMLFileLinkScanner;
       BS : PBufStream;
       Re : Word;
       params : array[0..0] of pointer;
       dir    : searchrec;
       r : trect;

    begin
       r.assign(10,10,70,15);
       indexdlg:=new(phtmlindexdialog,init(r,'Creating HTML index file, please wait ...'));
       desktop^.insert(indexdlg);
 { warning FIXME !!!!, don't know what is to fix here ... PM }
       New(LS, Init(DirOf(FileName)));
       LS^.ProcessDocument(FileName,[soSubDocsOnly]);
       if LS^.GetDocumentCount=0 then
         begin
           params[0]:=@filename;
           MessageBox('Problem creating help index %1, aborting',@params,
                  mferror+mfokbutton);
         end
       else
         begin
           FileName:=DirAndNameOf(FileName)+HTMLIndexExt;
           findfirst(filename,AnyFile,dir);
           if doserror=0 then
             begin
                params[0]:=@filename;
                Re:=MessageBox('Help index %s already exists, overwrite it?',@params,
                  mfinformation+mfyesbutton+mfnobutton);
             end
           else
             Re:=cmYes;
           if Re<>cmNo then
           begin
             New(BS, Init(FileName, stCreate, 4096));
             if Assigned(BS)=false then
               begin
                  MessageBox('Error while writing help index! '+
                    'No help index is created',@params,
                    mferror+mfokbutton);
                  Re:=cmCancel;
               end
             else
               begin
                 LS^.StoreDocuments(BS^);
                 if BS^.Status<>stOK then
                   begin
                      MessageBox('Error while writing help index!'#13+
                        'No help index is created',@params,
                        mferror+mfokbutton);
                      Re:=cmCancel;
                   end;
                 Dispose(BS, Done);
               end;
           end;
         end;
       Dispose(LS, Done);
       desktop^.delete(indexdlg);
       dispose(indexdlg,done);
    end;

{*****************************************************************************
                          Writing of fpc.cfg
*****************************************************************************}

  procedure writedefcfg(const fn:string;const cfgdata : tcfgarray;count : longint;const targetname : string);
    var
      t      : text;
      i      : longint;
      s      : string;
      dir    : searchrec;
      params : array[0..0] of pointer;
      d : dirstr;
      n : namestr;
      e : extstr;
    begin
    { already exists }
      findfirst(fn,AnyFile,dir);
      if doserror=0 then
       begin
         params[0]:=@fn;
         if MessageBox('Config %s already exists, continue writing default config?',@params,
                       mfinformation+mfyesbutton+mfnobutton)=cmNo then
           exit;
       end;
    { create directory }
      fsplit(fn,d,n,e);
      createdir(d);
    { create the fpc.cfg }
      assign(t,fn);
      {$I-}
       rewrite(t);
      {$I+}
      if ioresult<>0 then
       begin
         params[0]:=@fn;
         MessageBox(#3'A config not written.'#13#3'%s'#13#3'couldn''t be created',@params,mfinformation+mfokbutton);
         exit;
       end;
      for i:=1 to count do
       if assigned(cfgdata[i]) then
         begin
           s:=cfgdata[i]^;
           Replace(s,'%basepath%',data.basepath);
           Replace(s,'%targetname%',targetname);
           if pos('-',targetname)=0 then
             begin
               Replace(s,'%targetos%',targetname);
               Replace(s,'%fpctargetmacro%','$FPCOS')
             end
           else
             begin
               Replace(s,'%targetos%',Copy(targetname,pos('-',targetname)+1,255));
               Replace(s,'%fpctargetmacro%','$FPCTARGET');
             end;
           writeln(t,s);
         end
       else
         writeln(t,'');
      close(t);
    end;


{*****************************************************************************
                               TUnZipDialog
*****************************************************************************}

  constructor tunzipdialog.Init(var Bounds: TRect; ATitle: TTitleStr);
    var
      r : trect;
    begin
      inherited init(bounds,atitle);
      Options:=Options or ofCentered;
(*      R.Assign (11, 4, 38, 6);*)
      R.Assign (1, 4,bounds.B.X-Bounds.A.X-2, 6);
      filetext:=new(pstatictext,init(r,#3'File: '));
      insert(filetext);
      R.Assign (1, 7,bounds.B.X-Bounds.A.X-2, 9);
      extractfiletext:=new(pstatictext,init(r,#3' '));
      insert(extractfiletext);
    end;

{$IFNDEF DLL}
  procedure UnzipCheckFn (Retcode: longint; Rec: pReportRec );{$ifdef Delphi32}STDCALL;{$endif}
  {$ifndef fpc}{$IFNDEF BIT32} FAR;{$ENDIF BIT32}{$endif}
  var
    name : string;
  begin
    case Rec^.Status of
     unzip_starting:
         UnzipErr := 0;
     file_starting:
        begin
         with UnzDlg^.extractfiletext^ do
         begin
          Disposestr(text);
          name:=Strpas(Rec^.FileName);
          UnzDlg^.currentfile:=name;
          Text:=NewStr(#3+name);
          DrawView;
         end;
        end;
     file_failure:
       UnzipErr := RetCode;
     file_unzipping:
        begin
         with UnzDlg^.FileText^ do
         begin
          Inc (StatusPos);
          if StatusPos > MaxStatusPos then StatusPos := 1;
          Text^ [Length (Text^)] := StatusChars [StatusPos];
          DrawView;
         end;
        end;
    end;
  end;
{$ENDIF}

  procedure tunzipdialog.do_unzip(s,topath : string);
    var
{$ifdef MAYBE_LFN}
      p : pathstr;
      n : namestr;
      e : extstr;
      islfn : boolean;
{$endif MAYBE_LFN}
      again : boolean;
      st2,fn,dir,wild : string;

    begin
       Disposestr(filetext^.text);
       filetext^.Text:=NewStr(#3'File: '+s + #13#3' ');
       filetext^.drawview;
       if not(file_exists(s,startpath)) then
         begin
            messagebox('File "'+s+'" missing for the selected installation. '+
                       'Installation hasn''t been completed.',nil,mferror+mfokbutton);
            WriteLog ('File "' + S +
                                   '" missing for the selected installation!');
            errorhalt;
         end;
{$IFNDEF DLL}
 {$IFDEF FPC}
       SetUnzipReportProc (@UnzipCheckFn);
 {$ELSE FPC}
       SetUnzipReportProc (UnzipCheckFn);
 {$ENDIF FPC}
{$ENDIF DLL}

       WriteLog ('Unpacking ' + AllFiles + ' from '
                                   + StartPath + DirSep + S + ' to ' + ToPath);
       repeat
         fn:=startpath+DirSep+s+#0;
         dir:=topath+#0;
         wild:=AllFiles + #0;
         again:=false;
         FileUnzipEx(@fn[1],@dir[1],@wild[1]);
         if (UnzipErr <> 0) and (UnzipErr <> 1) then
           begin
              if CreateLog then
                begin
                  WriteLn (Log, 'Error ', UnzipErr, ' while unpacking!');
                  Flush (Log);
                end;
              s:=GetZipErrorInfo(UnzipErr);
              { Str(UnzipErr,s);}
              st2:='';
              if UnzipErr=unzip_WriteErr then
                begin
{$ifdef MAYBE_LFN}
                  if not(locallfnsupport) then
                    begin
                      islfn:=false;
                      fsplit(currentfile,p,n,e);
                      if (length(n)>8) or (length(e)>4) or
                         (pos('.',n)>0) or (upper(p+n+e)<>upper(currentfile)) then
                        islfn:=true;
                      if islfn then
                        begin
                          WriteLog ('Error while extracting ' +
                           CurrentFile + ' because of missing LFN support,' +
                           LineEnding + '  skipping rest of ZIP file.');
                          messagebox('Error while extracting '+currentfile+
                            #13#3'because of missing lfn support'+
                            #13#3'skipping rest of zipfile '+s
                            ,nil,mferror+mfOkButton);
                          again:=false;
                          exit;
                        end;
                    end
                  else
{$endif MAYBE_LFN}
                    st2:=' Disk full?';
                end;
              if CreateLog then
                WriteLog ('Error (' + S + ') while extracting.' + ST2);
              if messagebox('Error (' + S + ') while extracting.'+st2+#13+
                            #13#3'Try again?',nil,mferror+mfyesbutton+mfnobutton)=cmYes then
               again:=true
              else
               errorhalt;
           end;
       until not again;
    end;


{*****************************************************************************
                               TEndDialog
*****************************************************************************}

  constructor tenddialog.init;
    var
      R       : TRect;
      P       : PStaticText;
      Control : PButton;
      YB: word;
{$IFNDEF UNIX}
      i : longint;
      S: string;
      WPath: boolean;
      MixedCasePath: boolean;
{$ENDIF}
{$IFDEF OS2}
      ErrPath: array [0..259] of char;
      Handle: longint;
      WLibPath: boolean;
    const
      EMXName: array [1..4] of char = 'EMX'#0;
      BFD2EName: array [1..6] of char = 'BFD2E'#0;
{$ENDIF}
    begin
      if haside then
        YB := 15
      else
        YB := 14;

{$IFNDEF UNIX}
      s:='';
      for i:=1 to cfg.packs do
       if cfg.pack[i].binsub<>'' then
        begin
          if s<>'' then
           s:=s+';';
          S := s+Data.BasePath + Cfg.pack[i].BinSub;
        end;
      if Pos (Upper (S), Upper (GetEnv ('PATH'))) = 0 then
       begin
         WPath := true;
         Inc (YB, 3);
       end
      else
       WPath := false;
      { look if path is set as Path,
        this leads to problems for mingw32 make PM }
      MixedCasePath:=false;
      for i:=1 to EnvCount do
        begin
          if Pos('PATH=',Upper(EnvStr(i)))=1 then
            if Pos('PATH=',EnvStr(i))<>1 then
              Begin
                MixedCasePath:=true;
                Inc(YB, 2);
              End;
        end;
  {$IFDEF OS2}
      if DosLoadModule (@ErrPath, SizeOf (ErrPath), @EMXName, Handle) = 0 then
       begin
         WLibPath := false;
         DosFreeModule (Handle);
         if DosLoadModule (@ErrPath, SizeOf (ErrPath), @BFD2EName, Handle) = 0 then
          begin
           WLibPath := false;
           DosFreeModule (Handle);
          end
         else
          begin
           WLibPath := true;
           Inc (YB, 2);
          end;
        end
       else
        begin
         WLibPath := true;
         Inc (YB, 2);
        end;
  {$ENDIF}
{$ENDIF}

      R.Assign(6, 6, 74, YB);
      inherited init(r,'Installation successful.');
      Options:=Options or ofCentered;

{$IFNDEF UNIX}
      if WPath then
       begin
         R.Assign(2, 3, 64, 5);
         P:=new(pstatictext,init(r,'Extend your PATH variable with '''+S+''''));
         insert(P);
       end;

  {$IFDEF OS2}
      if WLibPath then
       begin
         if WPath then
          S := 'and your LIBPATH with ''' + S
         else
          S := 'Extend your LIBPATH with ''' + S;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         System.Delete (S, Length (S) - 6, 7);
         S := S + 'dll''';
         R.Assign (2, YB - 15, 64, YB - 13);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         System.Delete (S, Length (S) - 3, 4);
         S := S + '\dll''';
         R.Assign (2, YB - 14, 64, YB - 12);
>>>>>>> graemeg/cpstrnew
         P := New (PStaticText, Init (R, S));
         Insert (P);
       end;
  {$ELSE OS2}
      if MixedCasePath then
       begin
         R.Assign(2, 5, 64, 6);
         P:=new(pstatictext,init(r,'You need to use setpath.bat file if you want to use Makefiles'));
         insert(P);
       end;
  {$ENDIF OS2}
{$ENDIF}

      R.Assign(2, YB - 13, 64, YB - 12);
      P:=new(pstatictext,init(r,'To compile files enter ''fpc [file]'''));
      insert(P);

      if haside then
        begin
           R.Assign(2, YB - 12, 64, YB - 10);
           P:=new(pstatictext,init(r,'To start the IDE (Integrated Development Environment) type ''fp'' at a command line prompt'));
           insert(P);
        end;

      R.Assign (29, YB - 9, 39, YB - 7);
      Control := New (PButton, Init (R,'~O~k', cmOK, bfDefault));
      Insert (Control);
    end;


{*****************************************************************************
                               TInstallDialog
*****************************************************************************}
{$ifdef MAYBE_LFN}
  var
     islfn : boolean;

  procedure lfnreport( Retcode : longint;Rec : pReportRec );

    var
       p : pathstr;
       n : namestr;
       e : extstr;

    begin
       fsplit(strpas(rec^.Filename),p,n,e);
       if (length(n)>8) or (length(e)>4) or
          (pos('.',n)>0) or (upper(p+n+e)<>upper(strpas(rec^.Filename))) then
         islfn:=true;
    end;

  function haslfn(const zipfile : string) : boolean;

    var
       buf : array[0..255] of char;

    begin
       strpcopy(buf,zipfile);
       islfn:=false;
{$ifdef FPC}
       ViewZip(buf,AllFiles,@lfnreport);
{$else FPC}
       ViewZip(buf,AllFiles,lfnreport);
{$endif FPC}
       haslfn:=islfn;
    end;
{$endif MAYBE_LFN}

  var
     AllFilesPresent : boolean;

  procedure presentreport( Retcode : longint;Rec : pReportRec );

    var
       st : string;
       f : file;
       size,time : longint;
       p : pathstr;
       n : namestr;
       e : extstr;

    begin
       if not ALLFilesPresent then
         exit;
       st:=Data.BasePath+strpas(rec^.Filename);
       fsplit(st,p,n,e);

       if not file_exists(n+e,p) then
         AllFilesPresent:=false
       else
         begin
           Assign(f,st);
           Reset(f,1);
           if IOresult<>0 then
             begin
               ALLfilesPresent:=false;
               exit;
             end;
           GetFtime(f,time);
           size:=FileSize(f);
           if (rec^.Time<>time) or (rec^.size<>size) then
             ALLFilesPresent:=false;
           close(f);
         end;
    end;

  function AreAllFilesPresent(const zipfile : string) : boolean;

    var
       buf : array[0..255] of char;

    begin
       strpcopy(buf,zipfile);
       AllFilesPresent:=true;
{$ifdef FPC}
       ViewZip(buf,AllFiles,@presentreport);
{$else FPC}
       ViewZip(buf,AllFiles,presentreport);
{$endif FPC}
       AreAllFilesPresent:=AllFilesPresent;
    end;

  constructor tinstalldialog.init;
    const
       width = 76;
       height = 20;
       x1 = (79-width) div 2;
       y1 = (23-height) div 2;
       x2 = x1+width;
       y2 = y1+height;
    var
       tabr,tabir,r : trect;
       packmask : array[1..maxpacks] of longint;
       enabmask : array[1..maxpacks] of longint;
       i,line,j : integer;
       items : array[1..maxpacks] of psitem;
       f : pview;
       found : boolean;
       okbut,cancelbut : pbutton;
       firstitem : array[1..maxpacks] of integer;
       packcbs : array[1..maxpacks] of pcheckboxes;
       packtd : ptabdef;
       labpath : plabel;
       ilpath : pspecialinputline;
       tab : ptab;
       titletext : pcoloredtext;
       labcfg : plabel;
       cfgcb : pcheckboxes;
       scrollbox: pscrollbox;
       sbr,sbsbr: trect;
       sbsb: pscrollbar;
       zipfile : string;
    begin
       f:=nil;
     { walk packages reverse and insert a newsitem for each, and set the mask }
       for j:=1 to cfg.packs do
        with cfg.pack[j] do
         begin
           firstitem[j]:=0;
           items[j]:=nil;
           packmask[j]:=0;
           enabmask[j]:=0;
           for i:=packages downto 1 do
            begin
              zipfile:='';
              if file_exists(package[i].zip,startpath) then
               zipfile:=startpath+DirSep+package[i].zip
              else if file_exists(package[i].zipshort,startpath) then
               begin
                 zipfile:=startpath+DirSep+package[i].zipshort;
                 { update package to replace the full zipname with the short name }
                 package[i].zip:=package[i].zipshort;
               end;
              if zipfile<>'' then
               begin
                 { get diskspace required }
                 package[i].diskspace:=diskspaceN(zipfile);
{$ifdef MAYBE_LFN}
                 if not(locallfnsupport) then
                   begin
                      if not(haslfn(zipfile)) then
                        begin
                           items[j]:=newsitem(package[i].name+diskspacestr(package[i].diskspace),items[j]);
                           packmask[j]:=packmask[j] or packagemask(i);
                           enabmask[j]:=enabmask[j] or packagemask(i);
                           firstitem[j]:=i-1;
                           WriteLog ('Checking lfn usage for ' + zipfile + ' ... no lfn');
                        end
                      else
                        begin
                           items[j]:=newsitem(package[i].name+' (requires LFN support)',items[j]);
                           enabmask[j]:=enabmask[j] or packagemask(i);
                           firstitem[j]:=i-1;
                           WriteLog ('Checking lfn usage for ' + zipfile + ' ... uses lfn');
                        end;
                   end
                 else
{$endif MAYBE_LFN}
                   begin
                      items[j]:=newsitem(package[i].name+diskspacestr(package[i].diskspace)
{$ifdef DEBUG}
                                         +' ('+dotstr(i)+')'
{$endif DEBUG}
                                         ,items[j]);
                      packmask[j]:=packmask[j] or packagemask(i);
                      enabmask[j]:=enabmask[j] or packagemask(i);
                      firstitem[j]:=i-1;
                   end;
               end
              else
               items[j]:=newsitem(package[i].name
{$ifdef DEBUG}
                          +' ('+dotstr(i)+')'
{$endif DEBUG}
                           ,items[j]);
            end;
         end;

     { If no component found abort }
       found:=false;
       for j:=1 to cfg.packs do
        if packmask[j]<>0 then
          found:=true;
       if not found then
        begin
          messagebox('No components found to install, aborting.',nil,mferror+mfokbutton);
          if CreateLog then
            WriteLog ('No components found to install, aborting.');
          errorhalt;
        end;

       r.assign(x1,y1,x2,y2);
       inherited init(r,'');
       Options:=Options or ofCentered;
       GetExtent(R);
       R.Grow(-2,-1);
       Dec(R.B.Y,2);
       TabR.Copy(R);
       TabIR.Copy(R);
       TabIR.Grow(-2,-2);
       TabIR.Move(-2,0);

       {-------- General Sheets ----------}
       R.Copy(TabIR);
       r.move(0,1);
       r.b.x:=r.a.x+40;
       r.b.y:=r.a.y+1;
       new(titletext,init(r,cfg.title,$71));

       r.move(0,2);
       r.b.x:=r.a.x+40;
       new(labpath,init(r,'~B~ase path',f));
       r.move(0,1);
       r.b.x:=r.a.x+40;
       r.b.y:=r.a.y+1;
       new(ilpath,init(r,high(DirStr)));

       r.move(0,2);
       r.b.x:=r.a.x+40;
       new(labcfg,init(r,'Con~f~ig',f));
       r.move(0,1);
       r.b.x:=r.a.x+40;
       r.b.y:=r.a.y+1;
       new(cfgcb,init(r,newsitem('create fpc.cfg',nil)));
       data.cfgval:=1;

       {-------- Pack Sheets ----------}
       for j:=1 to cfg.packs do
        begin
          R.Copy(TabIR);
          if R.A.Y+cfg.pack[j].packages>R.B.Y then
            R.B.Y:=R.A.Y+cfg.pack[j].packages;
          new(packcbs[j],init(r,items[j]));
          if data.packmask[j]=high(sw_word) then
           data.packmask[j]:=packmask[j];
          packcbs[j]^.enablemask:={$ifdef DEV}$7fffffff{$else}enabmask[j]{$endif};
          packcbs[j]^.sel:=firstitem[j];
        end;

       {--------- Main ---------}
       packtd:=nil;
       sbr.assign(1,3,tabr.b.x-tabr.a.x-3,tabr.b.y-tabr.a.y-1);
       for j:=cfg.packs downto 1 do
       begin
         if (sbr.b.y-sbr.a.y)<cfg.pack[j].packages then
          begin
            sbsbr.assign(sbr.b.x,sbr.a.y,sbr.b.x+1,sbr.b.y);
            sbsb:=CreateIdScrollBar (sbsbr.a.x, sbsbr.a.y,sbsbr.b.y-sbsbr.a.y,j,false);
            sbsb^.SetRange(0,cfg.pack[j].packages-(sbsbr.b.y-sbsbr.a.y)-1);
            sbsb^.SetStep(5,1);
            //New(sbsb, init(sbsbr));
          end
         else
           sbsb:=nil;
         New(ScrollBox, Init(sbr, nil, sbsb));
         PackCbs[j]^.MoveTo(0,0);
         ScrollBox^.Insert(PackCbs[j]);

         packtd:=NewTabDef(
           cfg.pack[j].name,ScrollBox,
             NewTabItem(sbsb,
             NewTabItem(ScrollBox,
             nil)),
           packtd);
       end;

       New(Tab, Init(TabR,
         NewTabDef('~G~eneral',IlPath,
           NewTabItem(TitleText,
           NewTabItem(LabPath,
           NewTabItem(ILPath,
           NewTabItem(LabCfg,
           NewTabItem(CfgCB,
           nil))))),
         packtd)
       ));
       Tab^.GrowMode:=0;

       Insert(Tab);

       line:=tabr.b.y;
       r.assign((width div 2)-18,line,(width div 2)-4,line+2);
       new(okbut,init(r,'~C~ontinue',cmok,bfdefault));
       Insert(OkBut);

       r.assign((width div 2)+4,line,(width div 2)+14,line+2);
       new(cancelbut,init(r,'~Q~uit',cmcancel,bfnormal));
       Insert(CancelBut);

       Tab^.Select;
    end;

  procedure tinstalldialog.handleevent(var event : tevent);
    begin
       if event.what=evcommand then
         if event.command=cmquit then
           begin
              putevent(event);
              event.command:=cmCancel;
           end;
       inherited handleevent(event);
    end;


{*****************************************************************************
                               TSpecialInputLine
*****************************************************************************}

{ this should use AreAllFilesPresent if the base dir is changed...
 but what if the installer has already choosen which files he wants ... }
procedure TSpecialInputLine.GetData(var Rec);
begin
  inherited GetData(Rec);
end;

{*****************************************************************************
                                TApp
*****************************************************************************}

  const
     cmstart = 1000;

  procedure tapp.do_installdialog;
    var
       p    : pinstalldialog;
       p3   : penddialog;
       r    : trect;
       result,
       c    : word;
       i,j  : longint;
       found : boolean;
{$ifndef Unix}
       DSize,Space,ASpace : int64;
       S: DirStr;
{$endif}

    procedure doconfigwrite;

      var
         i : longint;

      begin
         for i:=1 to cfg.packs do
           begin
             if cfg.pack[i].defcfgfile<>'' then
               writedefcfg(data.basepath+cfg.pack[i].binsub+DirSep+cfg.pack[i].defcfgfile,cfg.defcfg,cfg.defcfgs,cfg.pack[i].targetname);
             if cfg.pack[i].setpathfile<>'' then
               writedefcfg(data.basepath+cfg.pack[i].binsub+DirSep+cfg.pack[i].setpathfile,cfg.defsetpath,cfg.defsetpaths,cfg.pack[i].targetname);
           end;
         if haside then
           begin
              for i:=1 to cfg.packs do
                if cfg.pack[i].defidecfgfile<>'' then
                 writedefcfg(data.basepath+cfg.pack[i].binsub+DirSep+cfg.pack[i].defidecfgfile,cfg.defidecfg,cfg.defidecfgs,cfg.pack[i].targetname);
              for i:=1 to cfg.packs do
                if cfg.pack[i].defideinifile<>'' then
                 writedefcfg(data.basepath+cfg.pack[i].binsub+DirSep+cfg.pack[i].defideinifile,cfg.defideini,cfg.defideinis,cfg.pack[i].targetname);
              if hashtmlhelp then
                writehlpindex(data.basepath+DirSep+cfg.DocSub+DirSep+cfg.helpidx);
           end;
      end;

    begin
      data.basepath:=cfg.basepath;
      data.cfgval:=0;
      for j:=1 to cfg.packs do
       data.packmask[j]:=high(sw_word);

      repeat
      { select components }
        p:=new(pinstalldialog,init);
        c:=executedialog(p,@data);
        if (c=cmok) then
          begin
            if Data.BasePath = '' then
              messagebox('Please, choose the directory for installation first.',nil,mferror+mfokbutton)
            else
             begin
               found:=false;
               for j:=1 to cfg.packs do
                if data.packmask[j]>0 then
                 found:=true;
               if found then
                begin
{$IFNDEF UNIX}
                { TH - check the available disk space here }
                  DSize := 0;
                  for j:=1 to cfg.packs do
                   with cfg.pack[j] do
                    begin
                      for i:=1 to packages do
                       begin
                         if data.packmask[j] and packagemask(i)<>0 then
                         begin
                          ASpace := package[i].diskspace;
                          if ASpace = -1 then
                            begin
                              MessageBox ('File ' + package[i].zip +
                                            ' is probably corrupted!', nil,
                                                        mferror + mfokbutton);
                              WriteLog ('File ' + package[i].zip +
                                            ' is probably corrupted!');
                            end
                              else Inc (DSize, ASpace);
                         end;
                       end;
                    end;
                  WriteLog ('Diskspace needed: ' + DotStr (DSize) + ' Kb');

                  S := FExpand (Data.BasePath);
                  if S [Length (S)] = DirSep then
                   Dec (S [0]);
                  Space := DiskFree (byte (Upcase(S [1])) - 64);
                  { -1 means that the drive is invalid }
                  if Space=-1 then
                    begin
                     WriteLog ('The drive ' + S [1] + ': is not valid');
                     if messagebox('The drive '+S[1]+': is not valid. Do you ' +
                                   'want to change the installation path?',nil,
                                   mferror+mfyesbutton+mfnobutton) = cmYes then
                      Continue;
                      Space:=0;
                    end;
                  Space := Space shr 10;
                  WriteLog ('Free space on drive ' + S [1] + ': ' +
                                                       DotStr (Space) + ' Kb');

                  if Space < DSize then
                   S := 'is not '
                  else
                   S := '';
                  if (Space < DSize + 500) then
                   begin
                     if S = '' then
                      S := 'might not be ';
                     if messagebox('There ' + S + 'enough space on the target ' +
                                   'drive for all the selected components. Do you ' +
                                   'want to change the installation path?',nil,
                                   mferror+mfyesbutton+mfnobutton) = cmYes then
                      Continue;
                   end;
{$ENDIF}
                  if createinstalldir(data.basepath) then
                   break;
                end
               else
                begin
                  { maybe only config }
                  if (data.cfgval and 1)<>0 then
                   begin
                     result:=messagebox('No components selected.'#13#13'Create a configfile ?',nil,
                                                mfinformation+mfyesbutton+mfnobutton);
                     if (result=cmYes) and createinstalldir(data.basepath) then
                       doconfigwrite;
                     exit;
                   end
                  else
                   begin
                     result:=messagebox('No components selected.'#13#13'Abort installation?',nil,
                                               mferror+mfyesbutton+mfnobutton);
                     if result=cmYes then
                      exit;
                   end;
                end;
             end;
          end
        else
          exit;
      until false;

    { extract packages }
      for j:=1 to cfg.packs do
       with cfg.pack[j] do
        begin
          r.assign(10,7,70,18);
          UnzDlg:=new(punzipdialog,init(r,'Extracting Packages'));
          desktop^.insert(UnzDlg);
          for i:=1 to packages do
           begin
             if data.packmask[j] and packagemask(i)<>0 then
               begin
                  UnzDlg^.do_unzip(package[i].zip,data.basepath);
                  { gather some information about the installed files }
                  if copy(package[i].zip,1,3)='ide' then
                    haside:=true;
                  if copy(package[i].zip,1,7)='doc-htm' then
                    begin
                      hashtmlhelp:=true;
                      { correct the fpctoc file name if .html files are used }
                      if package[i].zip='doc-html.zip' then
                        if copy(cfg.helpidx,length(cfg.helpidx)-3,4)='.htm' then
                          cfg.helpidx:=cfg.helpidx+'l';
                    end;
               end;
           end;
          desktop^.delete(UnzDlg);
          dispose(UnzDlg,done);
        end;

    { write config }
      if (data.cfgval and 1)<>0 then
        doconfigwrite;

    { show end message }
      p3:=new(penddialog,init);
      executedialog(p3,nil);
    end;


  procedure tapp.readcfg(const fn:string);
    var
      t    : text;
      i,j,k,
      line : longint;
      item,
      s,hs   : string;
      params : array[0..0] of pointer;

{$ifndef FPC}
      procedure readln(var t:text;var s:string);
      var
        c : char;
        i : longint;
      begin
        c:=#0;
        i:=0;
        while (not eof(t)) and (c<>#10) do
         begin
           read(t,c);
           if c<>#10 then
            begin
              inc(i);
              s[i]:=c;
            end;
         end;
        if (i>0) and (s[i]=#13) then
         dec(i);
        s[0]:=chr(i);
      end;
{$endif}

    begin
      assign(t,StartPath + DirSep + fn);
      {$I-}
       reset(t);
      {$I+}
      if ioresult<>0 then
       begin
         StartPath := GetProgDir;
         assign(t,StartPath + DirSep + fn);
         {$I-}
          reset(t);
         {$I+}
         if ioresult<>0 then
          begin
            params[0]:=@fn;
            messagebox('File %s not found!',@params,mferror+mfokbutton);
            WriteLog ('File "' + fn + '" not found!');
            errorhalt;
          end;
       end;
      line:=0;
      while not eof(t) do
       begin
         readln(t,s);
         inc(line);
         if (s<>'') and not(s[1] in ['#',';']) then
          begin
            i:=pos('=',s);
            if i>0 then
             begin
               item:=upper(Copy(s,1,i-1));
               system.delete(s,1,i);
               if item='VERSION' then
                cfg.version:=s
               else
                if item='TITLE' then
                 cfg.title:=s
               else
                if item='BASEPATH' then
                 cfg.basepath:=s
               else
                if item='HELPIDX' then
                   cfg.helpidx:=s
               else
                if item='DOCSUB' then
                   cfg.docsub:=s
               else
                if item='DEFAULTCFG' then
                 begin
                   repeat
                     readln(t,s);
                     if upper(s)='ENDCFG' then
                      break;
                     if cfg.defcfgs<maxdefcfgs then
                      begin
                        inc(cfg.defcfgs);
                        cfg.defcfg[cfg.defcfgs]:=newstr(s);
                      end;
                   until false;
                 end
               else
                if item='DEFAULTIDECFG' then
                 begin
                   repeat
                     readln(t,s);
                     if upper(s)='ENDCFG' then
                      break;
                     if cfg.defidecfgs<maxdefcfgs then
                      begin
                        inc(cfg.defidecfgs);
                        cfg.defidecfg[cfg.defidecfgs]:=newstr(s);
                      end;
                   until false;
                 end
               else
                if item='DEFAULTSETPATH' then
                 begin
                   repeat
                     readln(t,s);
                     if upper(s)='ENDCFG' then
                      break;
                     if cfg.defsetpaths<maxdefcfgs then
                      begin
                        inc(cfg.defsetpaths);
                        cfg.defsetpath[cfg.defsetpaths]:=newstr(s);
                      end;
                   until false;
                 end
               else
                if item='DEFAULTIDEINI' then
                 begin
                   repeat
                     readln(t,s);
                     if upper(s)='ENDCFG' then
                      break;
                     if cfg.defideinis<maxdefcfgs then
                      begin
                        inc(cfg.defideinis);
                        cfg.defideini[cfg.defideinis]:=newstr(s);
                      end;
                   until false;
                 end
               else
                if item='PACK' then
                 begin
                   inc(cfg.packs);
                   if cfg.packs>maxpacks then
                    begin
                      MessageBox ('Too many packs!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'Too many packs');
                        close(log);
                      end;
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].name:=s;
                 end
               else
                if item='CFGFILE' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'No pack set');
                        close(Log);
                      end;
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].defcfgfile:=s
                 end
               else
                if item='IDECFGFILE' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'No pack set');
                        Close(Log);
                      end;
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].defidecfgfile:=s
                 end
               else
                if item='SETPATHFILE' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'No pack set');
                        close(Log);
                      end;
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].setpathfile:=s
                 end
               else
                if item='IDEINIFILE' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'No pack set');
                        Close(Log);
                      end;
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].defideinifile:=s
                 end
               else
                if item='PPC386' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'No pack set');
                        Close(Log);
                      end;
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].ppc386:=s;
                 end
               else
                if item='BINSUB' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'No pack set');
                        Close(Log);
                      end;
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].binsub:=s;
                 end
               {else: Obsolete PM }
                { if item='FILECHECK' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                        WriteLn (Log, 'No pack set');
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].filechk:=s;
                 end }
               else
                if item='TARGETNAME' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'No pack set');
                        Close(Log);
                      end;
                      halt(1);
                    end;
                   cfg.pack[cfg.packs].targetname:=s;
                 end
               else
                if item='PACKAGE' then
                 begin
                   if cfg.packs=0 then
                    begin
                      MessageBox ('No pack set found!', nil,
                                                         mfError + mfOkButton);
                      if CreateLog then
                      begin
                        WriteLn (Log, 'No pack set');
                        Close(Log);
                      end;
                      halt(1);
                    end;
                   with cfg.pack[cfg.packs] do
                    begin
                      j:=pos(',',s);
                      if (j>0) and (packages<maxpackages) then
                       begin
                         inc(packages);
                         hs:=copy(s,1,j-1);
                         k:=pos('[',hs);
                         if (k>0) then
                          begin
                            package[packages].zip:=Copy(hs,1,k-1);
                            package[packages].zipshort:=Copy(hs,k+1,length(hs)-k-1);
                          end
                         else
                          package[packages].zip:=hs;
                         package[packages].name:=copy(s,j+1,255);
                       end;
                      package[packages].diskspace:=-1;
                    end;
                 end
             end;
          end;
       end;
      close(t);
    end;


  procedure tapp.checkavailpack;
    var
      i, j : longint;
      one_found : boolean;
    begin
    { check the packages }
      j:=0;
      while (j<cfg.packs) do
        begin
          inc(j);
          one_found:=false;
          {if cfg.pack[j].filechk<>'' then}
          for i:=1 to cfg.pack[j].packages do
            begin
              if file_exists(cfg.pack[j].package[i].zip,startpath) or
                 file_exists(cfg.pack[j].package[i].zipshort,startpath) then
                begin
                  one_found:=true;
                  break;
                end;
            end;

          if not one_found then
            begin
              { remove the package }
              move(cfg.pack[j+1],cfg.pack[j],sizeof(tpack)*(cfg.packs-j));
              dec(cfg.packs);
              dec(j);
            end;
        end;
    end;


  procedure tapp.initmenubar;
    var
       r : trect;
    begin
       getextent(r);
       r.b.y:=r.a.y+1;
       menubar:=new(pmenubar,init(r,newmenu(
          newsubmenu('Free Pascal Installer',hcnocontext,newmenu(nil
          ),
       nil))));
    end;


  procedure tapp.initstatusline;
    var
       R: TRect;
    begin
      GetExtent(R);
      R.A.Y := R.B.Y - 1;
<<<<<<< HEAD
<<<<<<< HEAD
      //R.B.X := R.B.X - 2;
=======
      R.B.X := R.B.X - 12;
>>>>>>> graemeg/fixes_2_2
=======
      R.B.X := R.B.X - 12;
>>>>>>> origin/fixes_2_2
      New(StatusLine,
        Init(R,
          NewStatusDef(0, $EFFF,nil,nil
          )
        )
      );
    end;


  procedure tapp.handleevent(var event : tevent);
    begin
       inherited handleevent(event);
       if event.what=evcommand then
         if event.command=cmstart then
           begin
              clearevent(event);
              do_installdialog;
              if successfull then
               begin
                 event.what:=evcommand;
                 event.command:=cmquit;
                 handleevent(event);
               end;
           end;
    end;

{$IFDEF DOSSTUB}
function CheckOS2: boolean;
var
 OwnName: PathStr;
 OwnDir: DirStr;
 Name: NameStr;
 Ext: ExtStr;
 DosV, W: word;
 P: PChar;
const
 Title: string [15] = 'FPC Installer'#0;
 RunBlock: TRunBlock = (Length: $32;
                        Dependent: 0;
                        Background: 0;
                        TraceLevel: 0;
                        PrgTitle: @Title [1];
                        PrgName: nil;
                        Args: nil;
                        TermQ: 0;
                        Environment: nil;
                        Inheritance: 0;
                        SesType: 2;
                        Icon: nil;
                        PgmHandle: 0;
                        PgmControl: 2;
                        Column: 0;
                        Row: 0;
                        Width: 80;
                        Height: 25);
begin
 CheckOS2 := false;
 asm
  mov ah, 30h
  int 21h
  xchg ah, al
  mov DosV, ax
  mov ax, 4010h
  int 2Fh
  cmp ax, 4010h
  jnz @0
  xor bx, bx
@0:
  mov W, bx
 end;
 if DosV > 3 shl 8 then
 begin
  OwnName := FExpand (ParamStr (0));
  FSplit (OwnName, OwnDir, Name, Ext);
  if (DosV >= 20 shl 8 + 10) and (W >= 20 shl 8 + 10) then
                       (* OS/2 version 2.1 or later running (double-checked) *)
  begin
   OwnName [Succ (byte (OwnName [0]))] := #0;
   RunBlock.PrgName := @OwnName [1];
   P := Ptr (PrefixSeg, $80);
   if PByte (P)^ <> 0 then
   begin
    Inc (P);
    RunBlock.Args := Ptr (PrefixSeg, $81);
   end;
   asm
    mov ax, 6400h
    mov bx, 0025h
    mov cx, 636Ch
    mov si, offset RunBlock
    int 21h
    jc @0
    mov DosV, 0
@0:
   end;
   CheckOS2 := DosV = 0;
  end;
 end;
end;
{$ENDIF}


procedure usagescreen;
begin
  writeln('FPC Installer ',installerversion,' ',installercopyright);
  writeln('Command line options:');
  writeln('  -l   create log file');
{$ifdef MAYBE_LFN}
  writeln('  --nolfn   force installation with short file names');
{$endif MAYBE_LFN}
  writeln;
  writeln('  -h   displays this help');
end;

var
  OldExit: pointer;

procedure NewExit;
begin
 ExitProc := OldExit;
 if CreateLog then
  begin
{$I-}
   if ErrorAddr <> nil then
    begin
     WriteLn (Log, 'Installer crashed with RTE ', ExitCode);
     Close (Log);
    end
   else
    if ExitCode <> 0 then
     begin
      WriteLn (Log, 'Installer ended with non-zero exit code ', ExitCode);
      Close (Log);
     end
{$I+}
  end;
end;


var
   i : longint;
{   vm : tvideomode;}
begin
   OldExit := ExitProc;
   ExitProc := @NewExit;
   { register objects for help streaming }
   RegisterWHTMLScan;
{$IFDEF OS2}
 { TH - no error boxes if checking an inaccessible disk etc. }
 {$IFDEF FPC}
   DosCalls.DosError (0);
 {$ELSE FPC}
  {$IFDEF VirtualPascal}
   OS2Base.DosError (ferr_DisableHardErr);
  {$ELSE VirtualPascal}
   BseDos.DosError (0);
  {$ENDIF VirtualPascal}
 {$ENDIF FPC}
{$ENDIF}
{$IFDEF DOSSTUB}
   if CheckOS2 then Halt;
{$ENDIF}
   createlog:=false;
{$ifdef MAYBE_LFN}
   locallfnsupport:=system.lfnsupport;
{$endif MAYBE_LFN}
   for i:=1 to paramcount do
     begin
        if paramstr(i)='-l' then
          createlog:=true
{$ifdef MAYBE_LFN}
        else if paramstr(i)='--nolfn' then
          begin
            locallfnsupport:=false;
{$ifdef GO32V2}
            { lfnsupport is a const in win32 RTL }
            system.lfnsupport:=locallfnsupport;
{$endif GO32V2}
          end
{$endif MAYBE_LFN}
        else if paramstr(i)='-h' then
          begin
             usagescreen;
             halt(0);
          end
        else
          begin
             usagescreen;
             halt(1);
          end;
     end;
   if createlog then
     begin
        assign(log,'install.log');
        rewrite(log);
{$ifdef MAYBE_LFN}
        if not(locallfnsupport) then
          WriteLog ('OS doesn''t have LFN support');
{$endif}
     end;
   getdir(0,startpath);
   successfull:=false;

   fillchar(cfg, SizeOf(cfg), 0);
   fillchar(data, SizeOf(data), 0);

   installapp.init;
{   vm.col:=80;
   vm.row:=25;
   vm.color:=true;
   installapp.SetScreenVideoMode(vm);
}
   FSplit (FExpand (ParamStr (0)), DStr, CfgName, EStr);

   installapp.readcfg(CfgName + CfgExt);
   installapp.checkavailpack;
{   installapp.readcfg(startpath+dirsep+cfgfile);}
{$ifdef GO32V2}
   if not(lfnsupport) then
     MessageBox('The operating system doesn''t support LFN (long file names),'+
       ' so some packages will get shorten filenames when installed',nil,mfinformation or mfokbutton);
{$endif}
   installapp.do_installdialog;
   installapp.done;
   if createlog then
     close(log);
end.
