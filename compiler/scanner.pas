{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the scanner part and handling of the switches

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
unit scanner;

{$i fpcdefs.inc}

interface

    uses
       cclasses,
       globtype,globals,constexp,version,tokens,
       verbose,comphook,
       finput,
       widestr;

    const
       max_include_nesting=32;
       max_macro_nesting=16;
       preprocbufsize=32*1024;


    type
       tcommentstyle = (comment_none,comment_tp,comment_oldtp,comment_delphi,comment_c);

       tscannerfile = class;

       preproctyp = (pp_ifdef,pp_ifndef,pp_if,pp_ifopt,pp_else,pp_elseif);

       tpreprocstack = class
          typ     : preproctyp;
          accept  : boolean;
          next    : tpreprocstack;
          name    : TIDString;
          line_nb : longint;
          fileindex : longint;
          constructor Create(atyp:preproctyp;a:boolean;n:tpreprocstack);
       end;

       tdirectiveproc=procedure;

       tdirectiveitem = class(TFPHashObject)
       public
          is_conditional : boolean;
          proc : tdirectiveproc;
          constructor Create(AList:TFPHashObjectList;const n:string;p:tdirectiveproc);
          constructor CreateCond(AList:TFPHashObjectList;const n:string;p:tdirectiveproc);
       end;

       // stack for replay buffers
       treplaystack = class
         token    : ttoken;
         idtoken  : ttoken;
         orgpattern,
         pattern  : string;
         cstringpattern: ansistring;
         patternw : pcompilerwidestring;
         settings : tsettings;
         tokenbuf : tdynamicarray;
         tokenbuf_needs_swapping : boolean;
         next     : treplaystack;
         constructor Create(atoken: ttoken;aidtoken:ttoken;
           const aorgpattern,apattern:string;const acstringpattern:ansistring;
           apatternw:pcompilerwidestring;asettings:tsettings;
           atokenbuf:tdynamicarray;change_endian:boolean;anext:treplaystack);
         destructor destroy;override;
       end;

       tcompile_time_predicate = function(var valuedescr: String) : Boolean;

       tspecialgenerictoken =
         (ST_LOADSETTINGS,
          ST_LINE,
          ST_COLUMN,
          ST_FILEINDEX,
          ST_LOADMESSAGES);

       { tscannerfile }
       tscannerfile = class
       private
         procedure do_gettokenpos(out tokenpos: longint; out filepos: tfileposinfo);
         procedure cachenexttokenpos;
         procedure setnexttoken;
         procedure savetokenpos;
         procedure restoretokenpos;
         procedure writetoken(t: ttoken);
         function readtoken : ttoken;
       public
          inputfile    : tinputfile;  { current inputfile list }
          inputfilecount : longint;

          inputbuffer,                { input buffer }
          inputpointer : pchar;
          inputstart   : longint;

          line_no,                    { line }
          lastlinepos  : longint;

          lasttokenpos,
          nexttokenpos : longint;     { token }
          lasttoken,
          nexttoken    : ttoken;

          oldlasttokenpos     : longint; { temporary saving/restoring tokenpos }
          oldcurrent_filepos,
          oldcurrent_tokenpos : tfileposinfo;


          replaytokenbuf,
          recordtokenbuf : tdynamicarray;

          { last settings we stored }
          last_settings : tsettings;
          last_message : pmessagestaterecord;
          { last filepos we stored }
          last_filepos,
          { if nexttoken<>NOTOKEN, then nexttokenpos holds its filepos }
          next_filepos   : tfileposinfo;

          comment_level,
          yylexcount     : longint;
          ignoredirectives : TFPHashList; { ignore directives, used to give warnings only once }
          preprocstack   : tpreprocstack;
          replaystack    : treplaystack;

          preproc_pattern : string;
          preproc_token   : ttoken;

          { true, if we are parsing preprocessor expressions }
          in_preproc_comp_expr : boolean;
          { true if tokens must be converted to opposite endianess}
          change_endian_for_replay : boolean;

          constructor Create(const fn:string; is_macro: boolean = false);
          destructor Destroy;override;
        { File buffer things }
          function  openinputfile:boolean;
          procedure closeinputfile;
          function  tempopeninputfile:boolean;
          procedure tempcloseinputfile;
          procedure saveinputfile;
          procedure restoreinputfile;
          procedure firstfile;
          procedure nextfile;
          procedure addfile(hp:tinputfile);
          procedure reload;
          { replaces current token with the text in p }
          procedure substitutemacro(const macname:string;p:pchar;len,line,fileindex:longint);
        { Scanner things }
          procedure gettokenpos;
          procedure inc_comment_level;
          procedure dec_comment_level;
          procedure illegal_char(c:char);
          procedure end_of_file;
          procedure checkpreprocstack;
          procedure poppreprocstack;
          procedure ifpreprocstack(atyp:preproctyp;compile_time_predicate:tcompile_time_predicate;messid:longint);
          procedure elseifpreprocstack(compile_time_predicate:tcompile_time_predicate);
          procedure elsepreprocstack;
          procedure popreplaystack;
          function replay_stack_depth:longint;
          procedure handleconditional(p:tdirectiveitem);
          procedure handledirectives;
          procedure linebreak;
          procedure recordtoken;
          procedure startrecordtokens(buf:tdynamicarray);
          procedure stoprecordtokens;
          function is_recording_tokens:boolean;
          procedure replaytoken;
          procedure startreplaytokens(buf:tdynamicarray; change_endian:boolean);
          { bit length asizeint is target depend }
          procedure tokenwritesizeint(val : asizeint);
          procedure tokenwritelongint(val : longint);
          procedure tokenwritelongword(val : longword);
          procedure tokenwriteword(val : word);
          procedure tokenwriteshortint(val : shortint);
          procedure tokenwriteset(var b;size : longint);
          procedure tokenwriteenum(var b;size : longint);
          function  tokenreadsizeint : asizeint;
          procedure tokenwritesettings(var asettings : tsettings; var size : asizeint);
          { longword/longint are 32 bits on all targets }
          { word/smallint are 16-bits on all targest }
          function  tokenreadlongword : longword;
          function  tokenreadword : word;
          function  tokenreadlongint : longint;
          function  tokenreadsmallint : smallint;
          { short int is one a signed byte }
          function  tokenreadshortint : shortint;
          function  tokenreadbyte : byte;
          { This one takes the set size as an parameter }
          procedure tokenreadset(var b;size : longint);
          function  tokenreadenum(size : longint) : longword;

          procedure tokenreadsettings(var asettings : tsettings; expected_size : asizeint);
          procedure readchar;
          procedure readstring;
          procedure readnumber;
          function  readid:string;
          function  readval:longint;
          function  readcomment(include_special_char: boolean = false):string;
          function  readquotedstring:string;
          function  readstate:char;
          function  readoptionalstate(fallback:char):char;
          function  readstatedefault:char;
          procedure skipspace;
          procedure skipuntildirective;
          procedure skipcomment(read_first_char:boolean);
          procedure skipdelphicomment;
          procedure skipoldtpcomment(read_first_char:boolean);
          procedure readtoken(allowrecordtoken:boolean);
          function  readpreproc:ttoken;
          function  readpreprocint(var value:int64;const place:string):boolean;
          function  asmgetchar:char;
       end;

{$ifdef PREPROCWRITE}
       tpreprocfile=class
         f   : text;
         buf : pointer;
         spacefound,
         eolfound : boolean;
         constructor create(const fn:string);
         destructor  destroy; override;
         procedure Add(const s:string);
         procedure AddSpace;
       end;
{$endif PREPROCWRITE}

    var
        { read strings }
        c              : char;
        orgpattern,
        pattern        : string;
        cstringpattern : ansistring;
        patternw       : pcompilerwidestring;

        { token }
        token,                        { current token being parsed }
        idtoken    : ttoken;          { holds the token if the pattern is a known word }

        current_scanner : tscannerfile;  { current scanner in use }

        current_commentstyle : tcommentstyle; { needed to use read_comment from directives }
{$ifdef PREPROCWRITE}
        preprocfile     : tpreprocfile;  { used with only preprocessing }
{$endif PREPROCWRITE}

    type
        tdirectivemode = (directive_all, directive_turbo, directive_mac);

    procedure AddDirective(const s:string; dm: tdirectivemode; p:tdirectiveproc);
    procedure AddConditional(const s:string; dm: tdirectivemode; p:tdirectiveproc);

    procedure InitScanner;
    procedure DoneScanner;

    { To be called when the language mode is finally determined }
    Function SetCompileMode(const s:string; changeInit: boolean):boolean;
    Function SetCompileModeSwitch(s:string; changeInit: boolean):boolean;
    procedure SetAppType(NewAppType:tapptype);

implementation

    uses
      SysUtils,
      cutils,cfileutl,
      systems,
      switches,
      symbase,symtable,symtype,symsym,symconst,symdef,defutil,
      { This is needed for tcputype }
      cpuinfo,
      fmodule,fppu,
      { this is needed for $I %CURRENTROUTINE%}
      procinfo
{$if FPC_FULLVERSION<20700}
      ,ccharset
{$endif}
      ;

    var
      { dictionaries with the supported directives }
      turbo_scannerdirectives : TFPHashObjectList;     { for other modes }
      mac_scannerdirectives   : TFPHashObjectList;     { for mode mac }


{*****************************************************************************
                              Helper routines
*****************************************************************************}

    const
      { use any special name that is an invalid file name to avoid problems }
      preprocstring : array [preproctyp] of string[7]
        = ('$IFDEF','$IFNDEF','$IF','$IFOPT','$ELSE','$ELSEIF');

    function is_keyword(const s:string):boolean;
      var
        low,high,mid : longint;
      begin
        if not (length(s) in [tokenlenmin..tokenlenmax]) or
           not (s[1] in ['a'..'z','A'..'Z']) then
         begin
           is_keyword:=false;
           exit;
         end;
        low:=ord(tokenidx^[length(s),s[1]].first);
        high:=ord(tokenidx^[length(s),s[1]].last);
        while low<high do
         begin
           mid:=(high+low+1) shr 1;
           if pattern<tokeninfo^[ttoken(mid)].str then
            high:=mid-1
           else
            low:=mid;
         end;
        is_keyword:=(pattern=tokeninfo^[ttoken(high)].str) and
                    ((tokeninfo^[ttoken(high)].keyword*current_settings.modeswitches)<>[]);
      end;


    Procedure HandleModeSwitches(switch: tmodeswitch; changeInit: boolean);
      begin
        { turn ansi/unicodestrings on by default ? (only change when this
          particular setting is changed, so that a random modeswitch won't
          change the state of $h+/$h-) }
        if switch in [m_none,m_default_ansistring,m_default_unicodestring] then
          begin
            if ([m_default_ansistring,m_default_unicodestring]*current_settings.modeswitches)<>[] then
              begin
                { can't have both ansistring and unicodestring as default }
                if switch=m_default_ansistring then
                  begin
                    exclude(current_settings.modeswitches,m_default_unicodestring);
                    if changeinit then
                      exclude(init_settings.modeswitches,m_default_unicodestring);
                  end
                else if switch=m_default_unicodestring then
                  begin
                    exclude(current_settings.modeswitches,m_default_ansistring);
                    if changeinit then
                      exclude(init_settings.modeswitches,m_default_ansistring);
                  end;
                { enable $h+ }
                include(current_settings.localswitches,cs_refcountedstrings);
                if changeinit then
                  include(init_settings.localswitches,cs_refcountedstrings);
                if m_default_unicodestring in current_settings.modeswitches then
                  begin
                    def_system_macro('FPC_UNICODESTRINGS');
                    def_system_macro('UNICODE');
                  end;
              end
            else
              begin
                exclude(current_settings.localswitches,cs_refcountedstrings);
                if changeinit then
                  exclude(init_settings.localswitches,cs_refcountedstrings);
                undef_system_macro('FPC_UNICODESTRINGS');
                undef_system_macro('UNICODE');
              end;
          end;

        { turn inline on by default ? }
        if switch in [m_none,m_default_inline] then
          begin
            if (m_default_inline in current_settings.modeswitches) then
             begin
               include(current_settings.localswitches,cs_do_inline);
               if changeinit then
                 include(init_settings.localswitches,cs_do_inline);
             end
            else
             begin
               exclude(current_settings.localswitches,cs_do_inline);
               if changeinit then
                 exclude(init_settings.localswitches,cs_do_inline);
             end;
          end;

        { turn on system codepage by default }
        if switch in [m_none,m_systemcodepage] then
          begin
            { both m_systemcodepage and specifying a code page via -FcXXX or
              "$codepage XXX" change current_settings.sourcecodepage. If
              we used -FcXXX and then have a sourcefile with "$mode objfpc",
              this routine will be called to disable m_systemcodepage (to ensure
              it's off in case it would have been set on the command line, or
              by a previous mode(switch).

              In that case, we have to ensure that we don't overwrite
              current_settings.sourcecodepage, as that would cancel out the
              -FcXXX. This is why we use two separate module switches
              (cs_explicit_codepage and cs_system_codepage) for the same setting
              (current_settings.sourcecodepage)
            }
            if m_systemcodepage in current_settings.modeswitches then
              begin
                { m_systemcodepage gets enabled -> disable any -FcXXX and
                  "codepage XXX" settings (exclude cs_explicit_codepage), and
                  overwrite the sourcecode page }
                current_settings.sourcecodepage:=DefaultSystemCodePage;
                if (current_settings.sourcecodepage<>CP_UTF8) and not cpavailable(current_settings.sourcecodepage) then
                  begin
                    Message2(scan_w_unavailable_system_codepage,IntToStr(current_settings.sourcecodepage),IntToStr(default_settings.sourcecodepage));
                    current_settings.sourcecodepage:=default_settings.sourcecodepage;
                  end;
                exclude(current_settings.moduleswitches,cs_explicit_codepage);
                include(current_settings.moduleswitches,cs_system_codepage);
                if changeinit then
                  begin
                    init_settings.sourcecodepage:=current_settings.sourcecodepage;
                    exclude(init_settings.moduleswitches,cs_explicit_codepage);
                    include(init_settings.moduleswitches,cs_system_codepage);
                  end;
              end
            else
              begin
                { m_systemcodepage gets disabled -> reset sourcecodepage only if
                  cs_explicit_codepage is not set (it may be set in the scenario
                  where -FcXXX was passed on the command line and then "$mode
                  fpc" is used, because then the caller of this routine will
                  set the "$mode fpc" modeswitches (which don't include
                  m_systemcodepage) and call this routine with m_none).

                  Or it can happen if -FcXXX was passed, and the sourcefile
                  contains "$modeswitch systemcodepage-" statement.

                  Since we unset cs_system_codepage if m_systemcodepage gets
                  activated, we will revert to the default code page if you
                  set a source file code page, then enable the systemcode page
                  and finally disable it again. We don't keep a stack of
                  settings, by design. The only thing we have to ensure is that
                  disabling m_systemcodepage if it wasn't on in the first place
                  doesn't overwrite the sourcecodepage }
                exclude(current_settings.moduleswitches,cs_system_codepage);
                if not(cs_explicit_codepage in current_settings.moduleswitches) then
                  current_settings.sourcecodepage:=default_settings.sourcecodepage;
                if changeinit then
                  begin
                    exclude(init_settings.moduleswitches,cs_system_codepage);
                    if not(cs_explicit_codepage in init_settings.moduleswitches) then
                      init_settings.sourcecodepage:=default_settings.sourcecodepage;
                  end;
              end;
          end;

{$ifdef i8086}
        { enable cs_force_far_calls when m_nested_procvars is enabled }
        if switch=m_nested_procvars then
          begin
            include(current_settings.localswitches,cs_force_far_calls);
            if changeinit then
              include(init_settings.localswitches,cs_force_far_calls);
          end;
{$endif i8086}
      end;


    Function SetCompileMode(const s:string; changeInit: boolean):boolean;
      var
        b : boolean;
        oldmodeswitches : tmodeswitches;
      begin
        oldmodeswitches:=current_settings.modeswitches;

        b:=true;
        if s='DEFAULT' then
          current_settings.modeswitches:=fpcmodeswitches
        else
         if s='DELPHI' then
          current_settings.modeswitches:=delphimodeswitches
        else
         if s='DELPHIUNICODE' then
          current_settings.modeswitches:=delphiunicodemodeswitches
        else
         if s='TP' then
          current_settings.modeswitches:=tpmodeswitches
        else
         if s='FPC' then begin
          current_settings.modeswitches:=fpcmodeswitches;
          { TODO: enable this for 2.3/2.9 }
          //  include(current_settings.localswitches, cs_typed_addresses);
        end else
         if s='OBJFPC' then begin
          current_settings.modeswitches:=objfpcmodeswitches;
          { TODO: enable this for 2.3/2.9 }
          //  include(current_settings.localswitches, cs_typed_addresses);
        end
{$ifdef gpc_mode}
        else if s='GPC' then
          current_settings.modeswitches:=gpcmodeswitches
{$endif}
        else
         if s='MACPAS' then
          current_settings.modeswitches:=macmodeswitches
        else
         if s='ISO' then
          current_settings.modeswitches:=isomodeswitches
        else
         if s='EXTENDEDPASCAL' then
          current_settings.modeswitches:=extpasmodeswitches
        else
         b:=false;

{$ifdef jvm}
          { enable final fields by default for the JVM targets }
          include(current_settings.modeswitches,m_final_fields);
{$endif jvm}

        if b and changeInit then
          init_settings.modeswitches := current_settings.modeswitches;

        if b then
         begin
           { resolve all postponed switch changes }
           flushpendingswitchesstate;

           HandleModeSwitches(m_none,changeinit);

           { turn on bitpacking and case checking for mode macpas and iso pascal,
             as well as extended pascal }
           if ([m_mac,m_iso,m_extpas] * current_settings.modeswitches <> []) then
             begin
               include(current_settings.localswitches,cs_bitpacking);
               include(current_settings.localswitches,cs_check_all_case_coverage);
               if changeinit then
                 begin
                   include(init_settings.localswitches,cs_bitpacking);
                   include(init_settings.localswitches,cs_check_all_case_coverage);
                 end;
             end;

           { support goto/label by default in delphi/tp7/mac/iso/extpas modes }
           if ([m_delphi,m_tp7,m_mac,m_iso,m_extpas] * current_settings.modeswitches <> []) then
             begin
               include(current_settings.moduleswitches,cs_support_goto);
               if changeinit then
                 include(init_settings.moduleswitches,cs_support_goto);
             end;

           { support pointer math by default in fpc/objfpc modes }
           if ([m_fpc,m_objfpc] * current_settings.modeswitches <> []) then
             begin
               include(current_settings.localswitches,cs_pointermath);
               if changeinit then
                 include(init_settings.localswitches,cs_pointermath);
             end
           else
             begin
               exclude(current_settings.localswitches,cs_pointermath);
               if changeinit then
                 exclude(init_settings.localswitches,cs_pointermath);
             end;

           { Default enum and set packing for delphi/tp7 }
           if (m_tp7 in current_settings.modeswitches) or
              (m_delphi in current_settings.modeswitches) then
             begin
               current_settings.packenum:=1;
               current_settings.setalloc:=1;
             end
           else if (m_mac in current_settings.modeswitches) then
             { compatible with Metrowerks Pascal }
             current_settings.packenum:=2
           else
             current_settings.packenum:=4;
           if changeinit then
             begin
               init_settings.packenum:=current_settings.packenum;
               init_settings.setalloc:=current_settings.setalloc;
             end;
{$if defined(i386) or defined(i8086)}
           { Default to intel assembler for delphi/tp7 on i386/i8086 }
           if (m_delphi in current_settings.modeswitches) or
              (m_tp7 in current_settings.modeswitches) then
             begin
{$ifdef i8086}
               current_settings.asmmode:=asmmode_i8086_intel;
{$else i8086}
               current_settings.asmmode:=asmmode_i386_intel;
{$endif i8086}
               if changeinit then
                 init_settings.asmmode:=current_settings.asmmode;
             end;
{$endif i386 or i8086}

           { Exception support explicitly turned on (mainly for macpas, to }
           { compensate for lack of interprocedural goto support)          }
           if (cs_support_exceptions in current_settings.globalswitches) then
             include(current_settings.modeswitches,m_except);

           { Default strict string var checking in TP/Delphi modes }
           if ([m_delphi,m_tp7] * current_settings.modeswitches <> []) then
             begin
               include(current_settings.localswitches,cs_strict_var_strings);
               if changeinit then
                 include(init_settings.localswitches,cs_strict_var_strings);
             end;

           { in delphi mode, excess precision is by default on }
           if ([m_delphi] * current_settings.modeswitches <> []) then
             begin
               include(current_settings.localswitches,cs_excessprecision);
               if changeinit then
                 include(init_settings.localswitches,cs_excessprecision);
             end;

{$ifdef i8086}
           { Do not force far calls in the TP mode by default, force it in other modes }
           if (m_tp7 in current_settings.modeswitches) then
             begin
               exclude(current_settings.localswitches,cs_force_far_calls);
               if changeinit then
                 exclude(init_settings.localswitches,cs_force_far_calls);
             end
           else
             begin
               include(current_settings.localswitches,cs_force_far_calls);
               if changeinit then
                 include(init_settings.localswitches,cs_force_far_calls);
             end;
{$endif i8086}

            { Undefine old symbol }
            if (m_delphi in oldmodeswitches) then
              undef_system_macro('FPC_DELPHI')
            else if (m_tp7 in oldmodeswitches) then
              undef_system_macro('FPC_TP')
            else if (m_objfpc in oldmodeswitches) then
              undef_system_macro('FPC_OBJFPC')
{$ifdef gpc_mode}
            else if (m_gpc in oldmodeswitches) then
              undef_system_macro('FPC_GPC')
{$endif}
            else if (m_mac in oldmodeswitches) then
              undef_system_macro('FPC_MACPAS')
            else if (m_iso in oldmodeswitches) then
              undef_system_macro('FPC_ISO')
            else if (m_extpas in oldmodeswitches) then
              undef_system_macro('FPC_EXTENDEDPASCAL');

            { define new symbol in delphi,objfpc,tp,gpc,macpas mode }
            if (m_delphi in current_settings.modeswitches) then
              def_system_macro('FPC_DELPHI')
            else if (m_tp7 in current_settings.modeswitches) then
              def_system_macro('FPC_TP')
            else if (m_objfpc in current_settings.modeswitches) then
              def_system_macro('FPC_OBJFPC')
{$ifdef gpc_mode}
            else if (m_gpc in current_settings.modeswitches) then
              def_system_macro('FPC_GPC')
{$endif}
            else if (m_mac in current_settings.modeswitches) then
              def_system_macro('FPC_MACPAS')
            else if (m_iso in current_settings.modeswitches) then
              def_system_macro('FPC_ISO')
            else if (m_extpas in current_settings.modeswitches) then
              def_system_macro('FPC_EXTENDEDPASCAL');
         end;

        SetCompileMode:=b;
      end;


    Function SetCompileModeSwitch(s:string; changeInit: boolean):boolean;
      var
        i : tmodeswitch;
        doinclude : boolean;
      begin
        s:=upper(s);

        { on/off? }
        doinclude:=true;
        case s[length(s)] of
          '+':
            s:=copy(s,1,length(s)-1);
          '-':
            begin
              s:=copy(s,1,length(s)-1);
              doinclude:=false;
            end;
        end;

        Result:=false;
        for i:=m_class to high(tmodeswitch) do
          if s=modeswitchstr[i] then
            begin
              { Objective-C is currently only supported for Darwin targets }
              if doinclude and
                 (i in [m_objectivec1,m_objectivec2]) and
                 not(target_info.system in systems_objc_supported) then
                begin
                  Message1(option_unsupported_target_for_feature,'Objective-C');
                  break;
                end;

              { Blocks supported? }
              if doinclude and
                 (i = m_blocks) and
                 not(target_info.system in systems_blocks_supported) then
                begin
                  Message1(option_unsupported_target_for_feature,'Blocks');
                  break;
                end;

              if changeInit then
                current_settings.modeswitches:=init_settings.modeswitches;
              Result:=true;
              if doinclude then
                begin
                  include(current_settings.modeswitches,i);
                  { Objective-C 2.0 support implies 1.0 support }
                  if (i=m_objectivec2) then
                    include(current_settings.modeswitches,m_objectivec1);
                  if (i in [m_objectivec1,m_objectivec2]) then
                    include(current_settings.modeswitches,m_class);
                end
              else
                begin
                  exclude(current_settings.modeswitches,i);
                  { Objective-C 2.0 support implies 1.0 support }
                  if (i=m_objectivec2) then
                    exclude(current_settings.modeswitches,m_objectivec1);
                  if (i in [m_objectivec1,m_objectivec2]) and
                     ([m_delphi,m_objfpc]*current_settings.modeswitches=[]) then
                    exclude(current_settings.modeswitches,m_class);
                end;

              { set other switches depending on changed mode switch }
              HandleModeSwitches(i,changeinit);

              if changeInit then
                init_settings.modeswitches:=current_settings.modeswitches;

              break;
            end;
      end;

    procedure SetAppType(NewAppType:tapptype);
      begin
{$ifdef i8086}
        if (target_info.system in [system_i8086_msdos,system_i8086_embedded]) and (apptype<>NewAppType) then
          begin
            if NewAppType=app_com then
              begin
                targetinfos[target_info.system]^.exeext:='.com';
                target_info.exeext:='.com';
              end
            else
              begin
                targetinfos[target_info.system]^.exeext:='.exe';
                target_info.exeext:='.exe';
              end;
          end;
{$endif i8086}
        if apptype in [app_cui,app_com] then
          undef_system_macro('CONSOLE');
        apptype:=NewAppType;
        if apptype in [app_cui,app_com] then
          def_system_macro('CONSOLE');
      end;
{*****************************************************************************
                           Conditional Directives
*****************************************************************************}

    procedure dir_else;
      begin
        current_scanner.elsepreprocstack;
      end;


    procedure dir_endif;
      begin
        current_scanner.poppreprocstack;
      end;

    function isdef(var valuedescr: String): Boolean;
      var
        hs    : string;
      begin
        current_scanner.skipspace;
        hs:=current_scanner.readid;
        valuedescr:= hs;
        if hs='' then
          Message(scan_e_error_in_preproc_expr);
        isdef:=defined_macro(hs);
      end;

    procedure dir_ifdef;
      begin
        current_scanner.ifpreprocstack(pp_ifdef,@isdef,scan_c_ifdef_found);
      end;

    function isnotdef(var valuedescr: String): Boolean;
      var
        hs    : string;
      begin
        current_scanner.skipspace;
        hs:=current_scanner.readid;
        valuedescr:= hs;
        if hs='' then
          Message(scan_e_error_in_preproc_expr);
        isnotdef:=not defined_macro(hs);
      end;

    procedure dir_ifndef;
      begin
        current_scanner.ifpreprocstack(pp_ifndef,@isnotdef,scan_c_ifndef_found);
      end;

    function opt_check(var valuedescr: String): Boolean;
      var
        hs    : string;
        state : char;
      begin
        opt_check:= false;
        current_scanner.skipspace;
        hs:=current_scanner.readid;
        valuedescr:= hs;
        if (length(hs)>1) then
          Message1(scan_w_illegal_switch,hs)
        else
          begin
            state:=current_scanner.ReadState;
            if state in ['-','+'] then
              opt_check:=CheckSwitch(hs[1],state)
            else
              Message(scan_e_error_in_preproc_expr);
          end;
      end;

    procedure dir_ifopt;
      begin
        flushpendingswitchesstate;
        current_scanner.ifpreprocstack(pp_ifopt,@opt_check,scan_c_ifopt_found);
      end;

    procedure dir_libprefix;
      var
        s : string;
      begin
        current_scanner.skipspace;
        if c <> '''' then
          Message2(scan_f_syn_expected, '''', c);
        s := current_scanner.readquotedstring;
        stringdispose(outputprefix);
        outputprefix := stringdup(s);
        with current_module do
         setfilename(paramfn, paramallowoutput);
      end;

    procedure dir_libsuffix;
      var
        s : string;
      begin
        current_scanner.skipspace;
        if c <> '''' then
          Message2(scan_f_syn_expected, '''', c);
        s := current_scanner.readquotedstring;
        stringdispose(outputsuffix);
        outputsuffix := stringdup(s);
        with current_module do
          setfilename(paramfn, paramallowoutput);
      end;

    procedure dir_extension;
      var
        s : string;
      begin
        current_scanner.skipspace;
        if c <> '''' then
          Message2(scan_f_syn_expected, '''', c);
        s := current_scanner.readquotedstring;
        if OutputFileName='' then
          OutputFileName:=InputFileName;
        OutputFileName:=ChangeFileExt(OutputFileName,'.'+s);
        with current_module do
          setfilename(paramfn, paramallowoutput);
      end;

{
Compile time expression type check
----------------------------------
Each subexpression returns its type to the caller, which then can
do type check.  Since data types of compile time expressions is
not well defined, the type system does a best effort. The drawback is
that some errors might not be detected.

Instead of returning a particular data type, a set of possible data types
are returned. This way ambigouos types can be handled.  For instance a
value of 1 can be both a boolean and and integer.

Booleans
--------

The following forms of boolean values are supported:
* C coded, that is 0 is false, non-zero is true.
* TRUE/FALSE for mac style compile time variables

Thus boolean mac compile time variables are always stored as TRUE/FALSE.
When a compile time expression is evaluated, they are then translated
to C coded booleans (0/1), to simplify for the expression evaluator.

Note that this scheme then also of support mac compile time variables which
are 0/1 but with a boolean meaning.

The TRUE/FALSE format is new from 22 august 2005, but the above scheme
means that units which is not recompiled, and thus stores
compile time variables as the old format (0/1), continue to work.

Short circuit evaluation
------------------------
For this to work, the part of a compile time expression which is short
circuited, should not be evaluated, while it still should be parsed.
Therefor there is a parameter eval, telling whether evaluation is needed.
In case not, the value returned can be arbitrary.
}

type

  { texprvalue }

  texprvalue = class
  private
    { we can't use built-in defs since they
      may be not created at the moment }
    class var
       sintdef,uintdef,booldef,strdef,setdef,realdef: tdef;
    class constructor createdefs;
    class destructor destroydefs;
  public
    consttyp: tconsttyp;
    value: tconstvalue;
    def: tdef;
    constructor create_const(c:tconstsym);
    constructor create_error;
    constructor create_ord(v: Tconstexprint);
    constructor create_int(v: int64);
    constructor create_uint(v: qword);
    constructor create_bool(b: boolean);
    constructor create_str(s: string);
    constructor create_set(ns: tnormalset);
    constructor create_real(r: bestreal);
    class function try_parse_number(s:string):texprvalue; static;
    class function try_parse_real(s:string):texprvalue; static;
    function evaluate(v:texprvalue;op:ttoken):texprvalue;
    procedure error(expecteddef, place: string);
    function isBoolean: Boolean;
    function isInt: Boolean;
    function asBool: Boolean;
    function asInt: Integer;
    function asInt64: Int64;
    function asStr: String;
    destructor destroy; override;
  end;

  class constructor texprvalue.createdefs;
    begin
      { do not use corddef etc here: this code is executed before those
        variables are initialised. Since these types are only used for
        compile-time evaluation of conditional expressions, it doesn't matter
        that we use the base types instead of the cpu-specific ones. }
      sintdef:=torddef.create(s64bit,low(int64),high(int64),false);
      uintdef:=torddef.create(u64bit,low(qword),high(qword),false);
      booldef:=torddef.create(pasbool1,0,1,false);
      strdef:=tstringdef.createansi(0,false);
      setdef:=tsetdef.create(sintdef,0,255,false);
      realdef:=tfloatdef.create(s80real,false);
    end;

  class destructor texprvalue.destroydefs;
    begin
      setdef.free;
      sintdef.free;
      uintdef.free;
      booldef.free;
      strdef.free;
      realdef.free;
    end;

  constructor texprvalue.create_const(c: tconstsym);
    begin
      consttyp:=c.consttyp;
      def:=c.constdef;
      case consttyp of
        conststring,
        constresourcestring:
          begin
            value.len:=c.value.len;
            getmem(value.valueptr,value.len+1);
            move(c.value.valueptr^,value.valueptr^,value.len+1);
          end;
        constwstring:
          begin
            initwidestring(value.valueptr);
            copywidestring(c.value.valueptr,value.valueptr);
          end;
        constreal:
          begin
            new(pbestreal(value.valueptr));
            pbestreal(value.valueptr)^:=pbestreal(c.value.valueptr)^;
          end;
        constset:
          begin
            new(pnormalset(value.valueptr));
            pnormalset(value.valueptr)^:=pnormalset(c.value.valueptr)^;
          end;
        constguid:
          begin
            new(pguid(value.valueptr));
            pguid(value.valueptr)^:=pguid(c.value.valueptr)^;
          end;
        else
          value:=c.value;
      end;
    end;

  constructor texprvalue.create_error;
    begin
      fillchar(value,sizeof(value),#0);
      consttyp:=constnone;
      def:=generrordef;
    end;

  constructor texprvalue.create_ord(v: Tconstexprint);
    begin
      fillchar(value,sizeof(value),#0);
      consttyp:=constord;
      value.valueord:=v;
      if v.signed then
        def:=sintdef
      else
        def:=uintdef;
    end;

  constructor texprvalue.create_int(v: int64);
    begin
      fillchar(value,sizeof(value),#0);
      consttyp:=constord;
      value.valueord:=v;
      def:=sintdef;
    end;

  constructor texprvalue.create_uint(v: qword);
    begin
      fillchar(value,sizeof(value),#0);
      consttyp:=constord;
      value.valueord:=v;
      def:=uintdef;
    end;

  constructor texprvalue.create_bool(b: boolean);
    begin
      fillchar(value,sizeof(value),#0);
      consttyp:=constord;
      value.valueord:=ord(b);
      def:=booldef;
    end;

  constructor texprvalue.create_str(s: string);
    var
      sp: pansichar;
      len: integer;
    begin
      fillchar(value,sizeof(value),#0);
      consttyp:=conststring;
      len:=length(s);
      getmem(sp,len+1);
      move(s[1],sp^,len+1);
      value.valueptr:=sp;
      value.len:=len;
      def:=strdef;
    end;

  constructor texprvalue.create_set(ns: tnormalset);
    begin
      fillchar(value,sizeof(value),#0);
      consttyp:=constset;
      new(pnormalset(value.valueptr));
      pnormalset(value.valueptr)^:=ns;
      def:=setdef;
    end;

  constructor texprvalue.create_real(r: bestreal);
    begin
      fillchar(value,sizeof(value),#0);
      consttyp:=constreal;
      new(pbestreal(value.valueptr));
      pbestreal(value.valueptr)^:=r;
      def:=realdef;
    end;

  class function texprvalue.try_parse_number(s:string):texprvalue;
    var
      ic: int64;
      qc: qword;
      code: integer;
    begin
      { try int64 }
      val(s,ic,code);
      if code=0 then
        result:=texprvalue.create_int(ic)
      else
        begin
          { try qword }
          val(s,qc,code);
          if code=0 then
            result:=texprvalue.create_uint(qc)
          else
            result:=try_parse_real(s);
        end;
    end;

  class function texprvalue.try_parse_real(s:string):texprvalue;
    var
      d: bestreal;
      code: integer;
    begin
      val(s,d,code);
      if code=0 then
        result:=texprvalue.create_real(d)
      else
        result:=nil;
    end;

  function texprvalue.evaluate(v:texprvalue;op:ttoken):texprvalue;

    function check_compatible: boolean;
      begin
        result:=(
                  (is_ordinal(v.def) or is_fpu(v.def)) and
                  (is_ordinal(def) or is_fpu(def))
                ) or
                (is_stringlike(v.def) and is_stringlike(def));
        if not result then
          Message2(type_e_incompatible_types,def.typename,v.def.typename);
      end;
    var
      lv,rv: tconstexprint;
      lvd,rvd: bestreal;
      lvs,rvs: string;
    begin
      case op of
        _OP_IN:
        begin
          if not is_set(v.def) then
            begin
              v.error('Set', 'IN');
              result:=texprvalue.create_error;
            end
          else
          if not is_ordinal(def) then
            begin
              error('Ordinal', 'IN');
              result:=texprvalue.create_error;
            end
          else
          if value.valueord.signed then
            result:=texprvalue.create_bool(value.valueord.svalue in pnormalset(v.value.valueptr)^)
          else
            result:=texprvalue.create_bool(value.valueord.uvalue in pnormalset(v.value.valueptr)^);
        end;
        _OP_NOT:
        begin
          if isBoolean then
            result:=texprvalue.create_bool(not asBool)
          else if is_ordinal(def) then
            begin
              result:=texprvalue.create_ord(value.valueord);
              result.def:=def;
              calc_not_ordvalue(result.value.valueord,result.def);
            end
          else
            begin
              error('Boolean', 'NOT');
              result:=texprvalue.create_error;
            end;
        end;
        _OP_OR:
        begin
          if isBoolean then
            if v.isBoolean then
              result:=texprvalue.create_bool(asBool or v.asBool)
            else
              begin
                v.error('Boolean','OR');
                result:=texprvalue.create_error;
              end
          else if is_ordinal(def) then
            if is_ordinal(v.def) then
              result:=texprvalue.create_ord(value.valueord or v.value.valueord)
            else
              begin
                v.error('Ordinal','OR');
                result:=texprvalue.create_error;
              end
          else
            begin
              error('Boolean','OR');
              result:=texprvalue.create_error;
            end;
        end;
        _OP_XOR:
        begin
          if isBoolean then
            if v.isBoolean then
              result:=texprvalue.create_bool(asBool xor v.asBool)
            else
              begin
                v.error('Boolean','XOR');
                result:=texprvalue.create_error;
              end
          else if is_ordinal(def) then
            if is_ordinal(v.def) then
              result:=texprvalue.create_ord(value.valueord xor v.value.valueord)
            else
              begin
                v.error('Ordinal','XOR');
                result:=texprvalue.create_error;
              end
          else
            begin
              error('Boolean','XOR');
              result:=texprvalue.create_error;
            end;
        end;
        _OP_AND:
        begin
          if isBoolean then
            if v.isBoolean then
              result:=texprvalue.create_bool(asBool and v.asBool)
            else
              begin
                v.error('Boolean','AND');
                result:=texprvalue.create_error;
              end
          else if is_ordinal(def) then
            if is_ordinal(v.def) then
              result:=texprvalue.create_ord(value.valueord and v.value.valueord)
            else
              begin
                v.error('Ordinal','AND');
                result:=texprvalue.create_error;
              end
          else
            begin
              error('Boolean','AND');
              result:=texprvalue.create_error;
            end;
        end;
        _EQ,_NE,_LT,_GT,_GTE,_LTE,_PLUS,_MINUS,_STAR,_SLASH,_OP_DIV,_OP_MOD,_OP_SHL,_OP_SHR:
        if check_compatible then
          begin
            if (is_ordinal(def) and is_ordinal(v.def)) then
              begin
                lv:=value.valueord;
                rv:=v.value.valueord;
                case op of
                  _EQ:
                    result:=texprvalue.create_bool(lv=rv);
                  _NE:
                    result:=texprvalue.create_bool(lv<>rv);
                  _LT:
                    result:=texprvalue.create_bool(lv<rv);
                  _GT:
                    result:=texprvalue.create_bool(lv>rv);
                  _GTE:
                    result:=texprvalue.create_bool(lv>=rv);
                  _LTE:
                    result:=texprvalue.create_bool(lv<=rv);
                  _PLUS:
                    result:=texprvalue.create_ord(lv+rv);
                  _MINUS:
                    result:=texprvalue.create_ord(lv-rv);
                  _STAR:
                    result:=texprvalue.create_ord(lv*rv);
                  _SLASH:
                    result:=texprvalue.create_real(lv/rv);
                  _OP_DIV:
                    result:=texprvalue.create_ord(lv div rv);
                  _OP_MOD:
                    result:=texprvalue.create_ord(lv mod rv);
                  _OP_SHL:
                    result:=texprvalue.create_ord(lv shl rv);
                  _OP_SHR:
                    result:=texprvalue.create_ord(lv shr rv);
                  else
                    begin
                      { actually we should never get here but this avoids a warning }
                      Message(parser_e_illegal_expression);
                      result:=texprvalue.create_error;
                    end;
                end;
              end
            else
            if (is_fpu(def) or is_ordinal(def)) and
               (is_fpu(v.def) or is_ordinal(v.def)) then
              begin
                if is_fpu(def) then
                  lvd:=pbestreal(value.valueptr)^
                else
                  lvd:=value.valueord;
                if is_fpu(v.def) then
                  rvd:=pbestreal(v.value.valueptr)^
                else
                  rvd:=v.value.valueord;
                case op of
                  _EQ:
                    result:=texprvalue.create_bool(lvd=rvd);
                  _NE:
                    result:=texprvalue.create_bool(lvd<>rvd);
                  _LT:
                    result:=texprvalue.create_bool(lvd<rvd);
                  _GT:
                    result:=texprvalue.create_bool(lvd>rvd);
                  _GTE:
                    result:=texprvalue.create_bool(lvd>=rvd);
                  _LTE:
                    result:=texprvalue.create_bool(lvd<=rvd);
                  _PLUS:
                    result:=texprvalue.create_real(lvd+rvd);
                  _MINUS:
                    result:=texprvalue.create_real(lvd-rvd);
                  _STAR:
                    result:=texprvalue.create_real(lvd*rvd);
                  _SLASH:
                    result:=texprvalue.create_real(lvd/rvd);
                  else
                    begin
                      Message(parser_e_illegal_expression);
                      result:=texprvalue.create_error;
                    end;
                end;
              end
            else
            begin
              lvs:=asStr;
              rvs:=v.asStr;
              case op of
                _EQ:
                  result:=texprvalue.create_bool(lvs=rvs);
                _NE:
                  result:=texprvalue.create_bool(lvs<>rvs);
                _LT:
                  result:=texprvalue.create_bool(lvs<rvs);
                _GT:
                  result:=texprvalue.create_bool(lvs>rvs);
                _GTE:
                  result:=texprvalue.create_bool(lvs>=rvs);
                _LTE:
                  result:=texprvalue.create_bool(lvs<=rvs);
                _PLUS:
                  result:=texprvalue.create_str(lvs+rvs);
                else
                  begin
                    Message(parser_e_illegal_expression);
                    result:=texprvalue.create_error;
                  end;
              end;
            end;
          end
        else
          result:=texprvalue.create_error;
        else
          result:=texprvalue.create_error;
      end;
    end;

  procedure texprvalue.error(expecteddef, place: string);
    begin
      Message3(scan_e_compile_time_typeerror,
               expecteddef,
               def.typename,
               place
              );
    end;

  function texprvalue.isBoolean: Boolean;
    var
      i: int64;
    begin
      result:=is_boolean(def);
      if not result and is_integer(def) then
        begin
          i:=asInt64;
          result:=(i=0)or(i=1);
        end;
    end;

  function texprvalue.isInt: Boolean;
    begin
      result:=is_integer(def);
    end;

  function texprvalue.asBool: Boolean;
    begin
      result:=value.valueord<>0;
    end;

  function texprvalue.asInt: Integer;
    begin
      result:=value.valueord.svalue;
    end;

  function texprvalue.asInt64: Int64;
    begin
      result:=value.valueord.svalue;
    end;

  function texprvalue.asStr: String;
    var
      b:byte;
    begin
      case consttyp of
        constord:
          result:=tostr(value.valueord);
        conststring,
        constresourcestring:
          SetString(result,pchar(value.valueptr),value.len);
        constreal:
          str(pbestreal(value.valueptr)^,result);
        constset:
          begin
            result:=',';
            for b:=0 to 255 do
              if b in pconstset(value.valueptr)^ then
                result:=result+tostr(b)+',';
          end;
        { error values }
        constnone:
          result:='';
        else
          internalerror(2013112801);
      end;
    end;

  destructor texprvalue.destroy;
    begin
      case consttyp of
        conststring,
        constresourcestring :
          freemem(value.valueptr,value.len+1);
        constwstring :
          donewidestring(pcompilerwidestring(value.valueptr));
        constreal :
          dispose(pbestreal(value.valueptr));
        constset :
          dispose(pnormalset(value.valueptr));
        constguid :
          dispose(pguid(value.valueptr));
        constord,
        { error values }
        constnone:
          ;
        else
          internalerror(2013112802);
      end;
      inherited destroy;
    end;

  const
    preproc_operators=[_EQ,_NE,_LT,_GT,_LTE,_GTE,_MINUS,_PLUS,_STAR,_SLASH,_OP_DIV,_OP_MOD,_OP_SHL,_OP_SHR,_OP_IN,_OP_AND,_OP_OR,_OP_XOR];

    function preproc_comp_expr:texprvalue;

        function preproc_sub_expr(pred_level:Toperator_precedence;eval:Boolean):texprvalue; forward;

        procedure preproc_consume(t:ttoken);
        begin
          if t<>current_scanner.preproc_token then
            Message(scan_e_preproc_syntax_error);
          current_scanner.preproc_token:=current_scanner.readpreproc;
        end;

        function try_consume_unitsym(var srsym:tsym;var srsymtable:TSymtable;out tokentoconsume:ttoken):boolean;
          var
            hmodule: tmodule;
            ns:ansistring;
            nssym:tsym;
          begin
            result:=false;
            tokentoconsume:=_ID;

            if assigned(srsym) and (srsym.typ in [unitsym,namespacesym]) then
              begin
                if not(srsym.owner.symtabletype in [staticsymtable,globalsymtable]) then
                  internalerror(200501154);
                { only allow unit.symbol access if the name was
                  found in the current module
                  we can use iscurrentunit because generic specializations does not
                  change current_unit variable }
                hmodule:=find_module_from_symtable(srsym.Owner);
                if not Assigned(hmodule) then
                  internalerror(201001120);
                if hmodule.unit_index=current_filepos.moduleindex then
                  begin
                    preproc_consume(_POINT);
                    current_scanner.skipspace;
                    if srsym.typ=namespacesym then
                      begin
                        ns:=srsym.name;
                        nssym:=srsym;
                        while assigned(srsym) and (srsym.typ=namespacesym) do
                          begin
                            { we have a namespace. the next identifier should be either a namespace or a unit }
                            searchsym_in_module(hmodule,ns+'.'+current_scanner.preproc_pattern,srsym,srsymtable);
                            if assigned(srsym) and (srsym.typ in [namespacesym,unitsym]) then
                              begin
                                ns:=ns+'.'+current_scanner.preproc_pattern;
                                nssym:=srsym;
                                preproc_consume(_ID);
                                current_scanner.skipspace;
                                preproc_consume(_POINT);
                                current_scanner.skipspace;
                              end;
                          end;
                        { check if there is a hidden unit with this pattern in the namespace }
                        if not assigned(srsym) and
                           assigned(nssym) and (nssym.typ=namespacesym) and assigned(tnamespacesym(nssym).unitsym) then
                          srsym:=tnamespacesym(nssym).unitsym;
                        if assigned(srsym) and (srsym.typ<>unitsym) then
                          internalerror(201108260);
                        if not assigned(srsym) then
                          begin
                            result:=true;
                            srsymtable:=nil;
                            exit;
                          end;
                      end;
                    case current_scanner.preproc_token of
                      _ID:
                        { system.char? (char=widechar comes from the implicit
                          uuchar unit -> override) }
                        if (current_scanner.preproc_pattern='CHAR') and
                           (tmodule(tunitsym(srsym).module).globalsymtable=systemunit) then
                          begin
                            if m_default_unicodestring in current_settings.modeswitches then
                              searchsym_in_module(tunitsym(srsym).module,'WIDECHAR',srsym,srsymtable)
                            else
                              searchsym_in_module(tunitsym(srsym).module,'ANSICHAR',srsym,srsymtable)
                          end
                        else
                          searchsym_in_module(tunitsym(srsym).module,current_scanner.preproc_pattern,srsym,srsymtable);
                      _STRING:
                        begin
                          { system.string? }
                          if tmodule(tunitsym(srsym).module).globalsymtable=systemunit then
                            begin
                              if cs_refcountedstrings in current_settings.localswitches then
                                begin
                                  if m_default_unicodestring in current_settings.modeswitches then
                                    searchsym_in_module(tunitsym(srsym).module,'UNICODESTRING',srsym,srsymtable)
                                  else
                                    searchsym_in_module(tunitsym(srsym).module,'ANSISTRING',srsym,srsymtable)
                                end
                              else
                                searchsym_in_module(tunitsym(srsym).module,'SHORTSTRING',srsym,srsymtable);
                              tokentoconsume:=_STRING;
                            end;
                        end
                      else
                        ;
                    end;
                  end
                else
                  begin
                    srsym:=nil;
                    srsymtable:=nil;
                  end;
                result:=true;
              end;
          end;

        procedure try_consume_nestedsym(var srsym:tsym;var srsymtable:TSymtable);
          var
            def:tdef;
            tokentoconsume:ttoken;
            found:boolean;
          begin
            found:=try_consume_unitsym(srsym,srsymtable,tokentoconsume);
            if found then
              begin
                preproc_consume(tokentoconsume);
                current_scanner.skipspace;
              end;
             while (current_scanner.preproc_token=_POINT) do
               begin
                 if assigned(srsym)and(srsym.typ=typesym) then
                   begin
                     def:=ttypesym(srsym).typedef;
                     if is_class_or_object(def) or is_record(def) or is_java_class_or_interface(def) then
                       begin
                         preproc_consume(_POINT);
                         current_scanner.skipspace;
                         if def.typ=objectdef then
                           found:=searchsym_in_class(tobjectdef(def),tobjectdef(def),current_scanner.preproc_pattern,srsym,srsymtable,[ssf_search_helper])
                         else
                           found:=searchsym_in_record(trecorddef(def),current_scanner.preproc_pattern,srsym,srsymtable);
                         if not found then
                           begin
                             Message1(sym_e_id_not_found,current_scanner.preproc_pattern);
                             exit;
                           end;
                         preproc_consume(_ID);
                         current_scanner.skipspace;
                       end
                     else
                       begin
                         Message(sym_e_type_must_be_rec_or_object_or_class);
                         exit;
                       end;
                   end
                 else
                   begin
                     Message(type_e_type_id_expected);
                     exit;
                   end;
               end;
          end;

        function preproc_substitutedtoken(searchstr:string;eval:Boolean):texprvalue;
        { Currently this parses identifiers as well as numbers.
          The result from this procedure can either be that the token
          itself is a value, or that it is a compile time variable/macro,
          which then is substituted for another value (for macros
          recursivelly substituted).}

        var
          hs: string;
          mac: tmacro;
          macrocount,
          len: integer;
        begin
          if not eval then
            begin
              result:=texprvalue.create_str(searchstr);
              exit;
            end;

          mac:=nil;
          { Substitue macros and compiler variables with their content/value.
            For real macros also do recursive substitution. }
          macrocount:=0;
          repeat
            mac:=tmacro(search_macro(searchstr));

            inc(macrocount);
            if macrocount>max_macro_nesting then
              begin
                Message(scan_w_macro_too_deep);
                break;
              end;

            if assigned(mac) and mac.defined then
              if assigned(mac.buftext) then
                begin
                  if mac.buflen>255 then
                    begin
                      len:=255;
                      Message(scan_w_macro_cut_after_255_chars);
                    end
                  else
                    len:=mac.buflen;
                  hs[0]:=char(len);
                  move(mac.buftext^,hs[1],len);
                  searchstr:=upcase(hs);
                  mac.is_used:=true;
                end
              else
                begin
                  Message1(scan_e_error_macro_lacks_value,searchstr);
                  break;
                end
            else
              break;

            if mac.is_compiler_var then
              break;
          until false;

          { At this point, result do contain the value. Do some decoding and
            determine the type.}
          result:=texprvalue.try_parse_number(searchstr);
          if not assigned(result) then
            begin
              if assigned(mac) and (searchstr='FALSE') then
                result:=texprvalue.create_bool(false)
              else if assigned(mac) and (searchstr='TRUE') then
                result:=texprvalue.create_bool(true)
              else if (m_mac in current_settings.modeswitches) and
                      (not assigned(mac) or not mac.defined) and
                      (macrocount = 1) then
                begin
                  {Errors in mode mac is issued here. For non macpas modes there is
                   more liberty, but the error will eventually be caught at a later stage.}
                  Message1(scan_e_error_macro_undefined,searchstr);
                  result:=texprvalue.create_str(searchstr); { just to have something }
                end
              else
                result:=texprvalue.create_str(searchstr);
            end;
        end;

        function preproc_factor(eval: Boolean):texprvalue;
        var
           hs,countstr,storedpattern: string;
           mac: tmacro;
           srsym : tsym;
           srsymtable : TSymtable;
           hdef : TDef;
           l : longint;
           hasKlammer: Boolean;
           exprvalue:texprvalue;
           ns:tnormalset;
        begin
          result:=nil;
          hasKlammer:=false;
           if current_scanner.preproc_token=_ID then
             begin
                if current_scanner.preproc_pattern='DEFINED' then
                  begin
                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    if current_scanner.preproc_token =_LKLAMMER then
                      begin
                        preproc_consume(_LKLAMMER);
                        current_scanner.skipspace;
                        hasKlammer:= true;
                      end
                    else if (m_mac in current_settings.modeswitches) then
                      hasKlammer:= false
                    else
                      Message(scan_e_error_in_preproc_expr);

                    if current_scanner.preproc_token =_ID then
                      begin
                        hs := current_scanner.preproc_pattern;
                        mac := tmacro(search_macro(hs));
                        if assigned(mac) and mac.defined then
                          begin
                            result:=texprvalue.create_bool(true);
                            mac.is_used:=true;
                          end
                        else
                          result:=texprvalue.create_bool(false);
                        preproc_consume(_ID);
                        current_scanner.skipspace;
                      end
                    else
                      Message(scan_e_error_in_preproc_expr);

                    if hasKlammer then
                      if current_scanner.preproc_token =_RKLAMMER then
                        preproc_consume(_RKLAMMER)
                      else
                        Message(scan_e_error_in_preproc_expr);
                  end
                else
                if (m_mac in current_settings.modeswitches) and (current_scanner.preproc_pattern='UNDEFINED') then
                  begin
                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    if current_scanner.preproc_token =_ID then
                      begin
                        hs := current_scanner.preproc_pattern;
                        mac := tmacro(search_macro(hs));
                        if assigned(mac) then
                          begin
                            result:=texprvalue.create_bool(false);
                            mac.is_used:=true;
                          end
                        else
                          result:=texprvalue.create_bool(true);
                        preproc_consume(_ID);
                        current_scanner.skipspace;
                      end
                    else
                      Message(scan_e_error_in_preproc_expr);
                  end
                else
                if (m_mac in current_settings.modeswitches) and (current_scanner.preproc_pattern='OPTION') then
                  begin
                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    if current_scanner.preproc_token =_LKLAMMER then
                      begin
                        preproc_consume(_LKLAMMER);
                        current_scanner.skipspace;
                      end
                    else
                      Message(scan_e_error_in_preproc_expr);

                    if not (current_scanner.preproc_token = _ID) then
                      Message(scan_e_error_in_preproc_expr);

                    hs:=current_scanner.preproc_pattern;
                    if (length(hs) > 1) then
                      {This is allowed in Metrowerks Pascal}
                      Message(scan_e_error_in_preproc_expr)
                    else
                      begin
                        if CheckSwitch(hs[1],'+') then
                          result:=texprvalue.create_bool(true)
                        else
                          result:=texprvalue.create_bool(false);
                      end;

                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    if current_scanner.preproc_token =_RKLAMMER then
                      preproc_consume(_RKLAMMER)
                    else
                      Message(scan_e_error_in_preproc_expr);
                  end
                else
                if current_scanner.preproc_pattern='SIZEOF' then
                  begin
                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    if current_scanner.preproc_token =_LKLAMMER then
                      begin
                        preproc_consume(_LKLAMMER);
                        current_scanner.skipspace;
                      end
                    else
                      Message(scan_e_preproc_syntax_error);

                    storedpattern:=current_scanner.preproc_pattern;
                    preproc_consume(_ID);
                    current_scanner.skipspace;

                    if eval then
                      if searchsym(storedpattern,srsym,srsymtable) then
                        begin
                          try_consume_nestedsym(srsym,srsymtable);
                          l:=0;
                          if assigned(srsym) then
                            case srsym.typ of
                              staticvarsym,
                              localvarsym,
                              paravarsym :
                                l:=tabstractvarsym(srsym).getsize;
                              typesym:
                                l:=ttypesym(srsym).typedef.size;
                              else
                                Message(scan_e_error_in_preproc_expr);
                            end;
                          result:=texprvalue.create_int(l);
                        end
                      else
                        Message1(sym_e_id_not_found,storedpattern);

                    if current_scanner.preproc_token =_RKLAMMER then
                      preproc_consume(_RKLAMMER)
                    else
                      Message(scan_e_preproc_syntax_error);
                  end
                else
                if current_scanner.preproc_pattern='HIGH' then
                  begin
                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    if current_scanner.preproc_token =_LKLAMMER then
                      begin
                        preproc_consume(_LKLAMMER);
                        current_scanner.skipspace;
                      end
                    else
                      Message(scan_e_preproc_syntax_error);

                    storedpattern:=current_scanner.preproc_pattern;
                    preproc_consume(_ID);
                    current_scanner.skipspace;

                    if eval then
                      if searchsym(storedpattern,srsym,srsymtable) then
                        begin
                          try_consume_nestedsym(srsym,srsymtable);
                          hdef:=nil;
                          hs:='';
                          l:=0;
                          if assigned(srsym) then
                            case srsym.typ of
                              staticvarsym,
                              localvarsym,
                              paravarsym :
                                hdef:=tabstractvarsym(srsym).vardef;
                              typesym:
                                hdef:=ttypesym(srsym).typedef;
                              else
                                Message(scan_e_error_in_preproc_expr);
                            end;
                          if assigned(hdef) then
                            begin
                              if hdef.typ=setdef then
                                hdef:=tsetdef(hdef).elementdef;
                              case hdef.typ of
                                orddef:
                                  with torddef(hdef).high do
                                    if signed then
                                      result:=texprvalue.create_int(svalue)
                                    else
                                      result:=texprvalue.create_uint(uvalue);
                                enumdef:
                                  result:=texprvalue.create_int(tenumdef(hdef).maxval);
                                arraydef:
                                  if is_open_array(hdef) or is_array_of_const(hdef) or is_dynamic_array(hdef) then
                                    Message(type_e_mismatch)
                                  else
                                    result:=texprvalue.create_int(tarraydef(hdef).highrange);
                                stringdef:
                                  if is_open_string(hdef) or is_ansistring(hdef) or is_wide_or_unicode_string(hdef) then
                                    Message(type_e_mismatch)
                                  else
                                    result:=texprvalue.create_int(tstringdef(hdef).len);
                                else
                                  Message(type_e_mismatch);
                              end;
                            end;
                        end
                      else
                        Message1(sym_e_id_not_found,storedpattern);

                    if current_scanner.preproc_token =_RKLAMMER then
                      preproc_consume(_RKLAMMER)
                    else
                      Message(scan_e_preproc_syntax_error);
                  end
                else
                if current_scanner.preproc_pattern='DECLARED' then
                  begin
                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    if current_scanner.preproc_token =_LKLAMMER then
                      begin
                        preproc_consume(_LKLAMMER);
                        current_scanner.skipspace;
                      end
                    else
                      Message(scan_e_error_in_preproc_expr);
                    if current_scanner.preproc_token =_ID then
                      begin
                        hs := upper(current_scanner.preproc_pattern);
                        preproc_consume(_ID);
                        current_scanner.skipspace;
                        if current_scanner.preproc_token in [_LT,_LSHARPBRACKET] then
                          begin
                            l:=1;
                            preproc_consume(current_scanner.preproc_token);
                            current_scanner.skipspace;
                            while current_scanner.preproc_token=_COMMA do
                              begin
                                inc(l);
                                preproc_consume(_COMMA);
                                current_scanner.skipspace;
                              end;
                            if not (current_scanner.preproc_token in [_GT,_RSHARPBRACKET]) then
                              Message(scan_e_error_in_preproc_expr)
                            else
                              preproc_consume(current_scanner.preproc_token);
                            str(l,countstr);
                            hs:=hs+'$'+countstr;
                          end
                        else
                          { special case: <> }
                          if current_scanner.preproc_token=_NE then
                            begin
                              hs:=hs+'$1';
                              preproc_consume(_NE);
                            end;
                        current_scanner.skipspace;
                        if searchsym(hs,srsym,srsymtable) then
                          begin
                            { TSomeGeneric<...> also adds a TSomeGeneric symbol }
                            if (sp_generic_dummy in srsym.symoptions) and
                                (srsym.typ=typesym) and
                                (
                                  { mode delphi}
                                  (ttypesym(srsym).typedef.typ in [undefineddef,errordef]) or
                                  { non-delphi modes }
                                  (df_generic in ttypesym(srsym).typedef.defoptions)
                                ) then
                              result:=texprvalue.create_bool(false)
                            else
                              result:=texprvalue.create_bool(true);
                          end
                        else
                          result:=texprvalue.create_bool(false);
                      end
                    else
                      Message(scan_e_error_in_preproc_expr);
                    if current_scanner.preproc_token =_RKLAMMER then
                      preproc_consume(_RKLAMMER)
                    else
                      Message(scan_e_error_in_preproc_expr);
                  end
                else
                if current_scanner.preproc_pattern='ORD' then
                  begin
                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    if current_scanner.preproc_token =_LKLAMMER then
                      begin
                        preproc_consume(_LKLAMMER);
                        current_scanner.skipspace;
                      end
                    else
                      Message(scan_e_preproc_syntax_error);

                    exprvalue:=preproc_factor(eval);
                    if eval then
                      begin
                        if is_ordinal(exprvalue.def) then
                          result:=texprvalue.create_int(exprvalue.asInt)
                        else
                          begin
                            exprvalue.error('Ordinal','ORD');
                            result:=texprvalue.create_int(0);
                          end;
                      end
                    else
                      result:=texprvalue.create_int(0);
                    exprvalue.free;
                    if current_scanner.preproc_token =_RKLAMMER then
                      preproc_consume(_RKLAMMER)
                    else
                      Message(scan_e_error_in_preproc_expr);
                  end
                else
                if current_scanner.preproc_pattern='NOT' then
                  begin
                    preproc_consume(_ID);
                    exprvalue:=preproc_factor(eval);
                    if eval then
                      result:=exprvalue.evaluate(nil,_OP_NOT)
                    else
                      result:=texprvalue.create_bool(false); {Just to have something}
                    exprvalue.free;
                  end
                else
                if (current_scanner.preproc_pattern='TRUE') then
                  begin
                    result:=texprvalue.create_bool(true);
                    preproc_consume(_ID);
                  end
                else
                if (current_scanner.preproc_pattern='FALSE') then
                  begin
                    result:=texprvalue.create_bool(false);
                    preproc_consume(_ID);
                  end
                else
                  begin
                    storedpattern:=current_scanner.preproc_pattern;
                    preproc_consume(_ID);
                    current_scanner.skipspace;
                    { first look for a macros/int/float }
                    result:=preproc_substitutedtoken(storedpattern,eval);
                    if eval and (result.consttyp=conststring) then
                      begin
                        if searchsym(storedpattern,srsym,srsymtable) then
                          begin
                            try_consume_nestedsym(srsym,srsymtable);
                            if assigned(srsym) then
                              case srsym.typ of
                                constsym:
                                  begin
                                    result.free;
                                    result:=texprvalue.create_const(tconstsym(srsym));
                                  end;
                                enumsym:
                                  begin
                                    result.free;
                                    result:=texprvalue.create_int(tenumsym(srsym).value);
                                  end;
                                else
                                  ;
                              end;
                          end
                        end
                      { skip id(<expr>) if expression must not be evaluated }
                      else if not(eval) and (result.consttyp=conststring) then
                        begin
                          if current_scanner.preproc_token =_LKLAMMER then
                            begin
                              preproc_consume(_LKLAMMER);
                              current_scanner.skipspace;

                              result:=preproc_factor(false);
                              if current_scanner.preproc_token =_RKLAMMER then
                                preproc_consume(_RKLAMMER)
                              else
                                Message(scan_e_error_in_preproc_expr);
                            end;
                        end;
                  end
             end
           else if current_scanner.preproc_token =_LKLAMMER then
             begin
                preproc_consume(_LKLAMMER);
                result:=preproc_sub_expr(opcompare,eval);
                preproc_consume(_RKLAMMER);
             end
           else if current_scanner.preproc_token = _LECKKLAMMER then
             begin
               preproc_consume(_LECKKLAMMER);
               ns:=[];
               while current_scanner.preproc_token in [_ID,_INTCONST] do
               begin
                 exprvalue:=preproc_factor(eval);
                 include(ns,exprvalue.asInt);
                 if current_scanner.preproc_token = _COMMA then
                   preproc_consume(_COMMA);
               end;
               // TODO Add check of setElemType
               preproc_consume(_RECKKLAMMER);
               result:=texprvalue.create_set(ns);
             end
           else if current_scanner.preproc_token = _INTCONST then
             begin
               result:=texprvalue.try_parse_number(current_scanner.preproc_pattern);
               if not assigned(result) then
                 begin
                   Message(parser_e_invalid_integer);
                   result:=texprvalue.create_int(1);
                 end;
               preproc_consume(_INTCONST);
             end
           else if current_scanner.preproc_token = _CSTRING then
             begin
               result:=texprvalue.create_str(current_scanner.preproc_pattern);
               preproc_consume(_CSTRING);
             end
           else if current_scanner.preproc_token = _REALNUMBER then
             begin
               result:=texprvalue.try_parse_real(current_scanner.preproc_pattern);
               if not assigned(result) then
                 begin
                   Message(parser_e_error_in_real);
                   result:=texprvalue.create_real(1.0);
                 end;
               preproc_consume(_REALNUMBER);
             end
           else
             Message(scan_e_error_in_preproc_expr);
           if not assigned(result) then
             result:=texprvalue.create_error;
        end;

        function preproc_sub_expr(pred_level:Toperator_precedence;eval:Boolean): texprvalue;
        var
          hs1,hs2: texprvalue;
          op: ttoken;
        begin
           if pred_level=highest_precedence then
             result:=preproc_factor(eval)
           else
             result:=preproc_sub_expr(succ(pred_level),eval);
          repeat
            op:=current_scanner.preproc_token;
            if (op in preproc_operators) and
               (op in operator_levels[pred_level]) then
             begin
               hs1:=result;
               preproc_consume(op);
               if (op=_OP_OR) and hs1.isBoolean and hs1.asBool then
                 begin
                   { stop evaluation the rest of expression }
                   result:=texprvalue.create_bool(true);
                   if pred_level=highest_precedence then
                     hs2:=preproc_factor(false)
                   else
                     hs2:=preproc_sub_expr(succ(pred_level),false);
                 end
               else if (op=_OP_AND) and hs1.isBoolean and not hs1.asBool then
                 begin
                   { stop evaluation the rest of expression }
                   result:=texprvalue.create_bool(false);
                   if pred_level=highest_precedence then
                     hs2:=preproc_factor(false)
                   else
                     hs2:=preproc_sub_expr(succ(pred_level),false);
                 end
               else
                 begin
                   if pred_level=highest_precedence then
                     hs2:=preproc_factor(eval)
                   else
                     hs2:=preproc_sub_expr(succ(pred_level),eval);
                   if eval then
                     result:=hs1.evaluate(hs2,op)
                   else
                     result:=texprvalue.create_bool(false); {Just to have something}
                 end;
               hs1.free;
               hs2.free;
             end
           else
             break;
          until false;
        end;

     begin
       current_scanner.in_preproc_comp_expr:=true;
       current_scanner.skipspace;
       { start preproc expression scanner }
       current_scanner.preproc_token:=current_scanner.readpreproc;
       preproc_comp_expr:=preproc_sub_expr(opcompare,true);
       current_scanner.in_preproc_comp_expr:=false;
     end;

    function boolean_compile_time_expr(var valuedescr: string): Boolean;
      var
        hs: texprvalue;
      begin
        hs:=preproc_comp_expr;
        if hs.isBoolean then
          result:=hs.asBool
        else
          begin
            hs.error('Boolean', 'IF or ELSEIF');
            result:=false;
          end;
        valuedescr:=hs.asStr;
        hs.free;
      end;

    procedure dir_if;
      begin
        current_scanner.ifpreprocstack(pp_if,@boolean_compile_time_expr, scan_c_if_found);
      end;

    procedure dir_elseif;
      begin
        current_scanner.elseifpreprocstack(@boolean_compile_time_expr);
      end;

    procedure dir_define_impl(macstyle: boolean);
      var
        hs  : string;
        bracketcount : longint;
        mac : tmacro;
        macropos : longint;
        macrobuffer : pmacrobuffer;
      begin
        current_scanner.skipspace;
        hs:=current_scanner.readid;
        if hs='' then
          begin
            Message(scan_e_emptymacroname);
            exit;
          end;
        mac:=tmacro(search_macro(hs));
        if not assigned(mac) or (mac.owner <> current_module.localmacrosymtable) then
          begin
            mac:=tmacro.create(hs);
            mac.defined:=true;
            current_module.localmacrosymtable.insert(mac);
          end
        else
          begin
            mac.defined:=true;
            mac.is_compiler_var:=false;
          { delete old definition }
            if assigned(mac.buftext) then
             begin
               freemem(mac.buftext,mac.buflen);
               mac.buftext:=nil;
             end;
          end;
        Message1(parser_c_macro_defined,mac.name);
        mac.is_used:=true;
        if (cs_support_macro in current_settings.moduleswitches) then
          begin
             current_scanner.skipspace;

             if not macstyle then
               begin
                 { may be a macro? }
                 if c <> ':' then
                   exit;
                 current_scanner.readchar;
                 if c <> '=' then
                   exit;
                 current_scanner.readchar;
                 current_scanner.skipspace;
               end;

             { key words are never substituted }
             if is_keyword(hs) then
               Message(scan_e_keyword_cant_be_a_macro);

             new(macrobuffer);
             macropos:=0;
             { parse macro, brackets are counted so it's possible
               to have a $ifdef etc. in the macro }
             bracketcount:=0;
             repeat
               case c of
                 '}' :
                   if (bracketcount=0) then
                    break
                   else
                    dec(bracketcount);
                 '{' :
                   inc(bracketcount);
                 #10,#13 :
                   current_scanner.linebreak;
                 #26 :
                   current_scanner.end_of_file;
               end;
               macrobuffer^[macropos]:=c;
               inc(macropos);
               if macropos>=maxmacrolen then
                 Message(scan_f_macro_buffer_overflow);
               current_scanner.readchar;
             until false;

             { free buffer of macro ?}
             if assigned(mac.buftext) then
               freemem(mac.buftext,mac.buflen);
             { get new mem }
             getmem(mac.buftext,macropos);
             mac.buflen:=macropos;
             { copy the text }
             move(macrobuffer^,mac.buftext^,macropos);
             dispose(macrobuffer);
          end
        else
          begin
           { check if there is an assignment, then we need to give a
             warning }
             current_scanner.skipspace;
             if c=':' then
              begin
                current_scanner.readchar;
                if c='=' then
                  Message(scan_w_macro_support_turned_off);
              end;
          end;
      end;

    procedure dir_define;
      begin
        dir_define_impl(false);
      end;

    procedure dir_definec;
      begin
        dir_define_impl(true);
      end;

    procedure dir_setc;
      var
        hs  : string;
        mac : tmacro;
        exprvalue: texprvalue;
      begin
        current_scanner.skipspace;
        hs:=current_scanner.readid;
        mac:=tmacro(search_macro(hs));
        if not assigned(mac) or
           (mac.owner <> current_module.localmacrosymtable) then
          begin
            mac:=tmacro.create(hs);
            mac.defined:=true;
            mac.is_compiler_var:=true;
            current_module.localmacrosymtable.insert(mac);
          end
        else
          begin
            mac.defined:=true;
            mac.is_compiler_var:=true;
          { delete old definition }
            if assigned(mac.buftext) then
             begin
               freemem(mac.buftext,mac.buflen);
               mac.buftext:=nil;
             end;
          end;
        Message1(parser_c_macro_defined,mac.name);
        mac.is_used:=true;

        { key words are never substituted }
        if is_keyword(hs) then
          Message(scan_e_keyword_cant_be_a_macro);

        { macro assignment can be both := and = }
        current_scanner.skipspace;
        if c=':' then
          current_scanner.readchar;
        if c='=' then
          begin
             current_scanner.readchar;
             exprvalue:=preproc_comp_expr;
             if not is_boolean(exprvalue.def) and
                not is_integer(exprvalue.def) then
               exprvalue.error('Boolean, Integer', 'SETC');
             hs:=exprvalue.asStr;

             if length(hs) <> 0 then
               begin
                 {If we are absolutely shure it is boolean, translate
                  to TRUE/FALSE to increase possibility to do future type check}
                 if exprvalue.isBoolean then
                   begin
                     if exprvalue.asBool then
                       hs:='TRUE'
                     else
                       hs:='FALSE';
                   end;
                 Message2(parser_c_macro_set_to,mac.name,hs);
                 { free buffer of macro ?}
                 if assigned(mac.buftext) then
                   freemem(mac.buftext,mac.buflen);
                 { get new mem }
                 getmem(mac.buftext,length(hs));
                 mac.buflen:=length(hs);
                 { copy the text }
                 move(hs[1],mac.buftext^,mac.buflen);
               end
             else
               Message(scan_e_preproc_syntax_error);
             exprvalue.free;
          end
        else
          Message(scan_e_preproc_syntax_error);
      end;


    procedure dir_undef;
      var
        hs  : string;
        mac : tmacro;
      begin
        current_scanner.skipspace;
        hs:=current_scanner.readid;
        mac:=tmacro(search_macro(hs));
        if not assigned(mac) or
           (mac.owner <> current_module.localmacrosymtable) then
          begin
             mac:=tmacro.create(hs);
             mac.defined:=false;
             current_module.localmacrosymtable.insert(mac);
          end
        else
          begin
             mac.defined:=false;
             mac.is_compiler_var:=false;
             { delete old definition }
             if assigned(mac.buftext) then
               begin
                  freemem(mac.buftext,mac.buflen);
                  mac.buftext:=nil;
               end;
          end;
        Message1(parser_c_macro_undefined,mac.name);
        mac.is_used:=true;
      end;

    procedure dir_include;

        function findincludefile(const path,name:TCmdStr;var foundfile:TCmdStr):boolean;
        var
          found  : boolean;
          hpath  : TCmdStr;
        begin
          (* look for the include file
           If path was absolute and specified as part of {$I } then
            1. specified path
           else
            1. path of current inputfile,current dir
            2. local includepath
            3. global includepath

            -- Check mantis #13461 before changing this *)
           found:=false;
           foundfile:='';
           hpath:='';
           if path_absolute(path) then
             begin
               found:=FindFile(name,path,true,foundfile);
             end
           else
             begin
               hpath:=current_scanner.inputfile.path+';'+CurDirRelPath(source_info);
               found:=FindFile(path+name, hpath,true,foundfile);
               if not found then
                 found:=current_module.localincludesearchpath.FindFile(path+name,true,foundfile);
               if not found  then
                 found:=includesearchpath.FindFile(path+name,true,foundfile);
             end;
           result:=found;
        end;

      var
        foundfile : TCmdStr;
        path,
        name,
        hs    : tpathstr;
        args  : string;
        hp    : tinputfile;
        found : boolean;
        macroIsString : boolean;
      begin
        current_scanner.skipspace;
        args:=current_scanner.readcomment;
        hs:=GetToken(args,' ');
        if hs='' then
         exit;
        if (hs[1]='%') then
         begin
         { case insensitive }
           hs:=upper(hs);
         { remove %'s }
           Delete(hs,1,1);
           if hs[length(hs)]='%' then
            Delete(hs,length(hs),1);
         { save old }
           path:=hs;
         { first check for internal macros }
           macroIsString:=true;
           case hs of
             'TIME':
               if timestr<>'' then
                 hs:=timestr
               else
                 hs:=gettimestr;
             'DATE':
               if datestr<>'' then
                 hs:=datestr
               else
                 hs:=getdatestr;
             'DATEYEAR':
               begin
                 hs:=tostr(startsystime.Year);
                 macroIsString:=false;
               end;
             'DATEMONTH':
               begin
                 hs:=tostr(startsystime.Month);
                 macroIsString:=false;
               end;
             'DATEDAY':
               begin
                 hs:=tostr(startsystime.Day);
                 macroIsString:=false;
               end;
             'TIMEHOUR':
               begin
                 hs:=tostr(startsystime.Hour);
                 macroIsString:=false;
               end;
             'TIMEMINUTE':
               begin
                 hs:=tostr(startsystime.Minute);
                 macroIsString:=false;
               end;
             'TIMESECOND':
               begin
                 hs:=tostr(startsystime.Second);
                 macroIsString:=false;
               end;
             'FILE':
               hs:=current_module.sourcefiles.get_file_name(current_filepos.fileindex);
             'LINE':
               hs:=tostr(current_filepos.line);
             'LINENUM':
               begin
                 hs:=tostr(current_filepos.line);
                 macroIsString:=false;
               end;
             'FPCVERSION':
               hs:=version_string;
             'FPCDATE':
               hs:=date_string;
             'FPCTARGET':
               hs:=target_cpu_string;
             'FPCTARGETCPU':
               hs:=target_cpu_string;
             'FPCTARGETOS':
               hs:=target_info.shortname;
             'CURRENTROUTINE':
               hs:=current_procinfo.procdef.procsym.RealName;
             else
               hs:=GetEnvironmentVariable(hs);
           end;
           if hs='' then
            Message1(scan_w_include_env_not_found,path);
           { make it a stringconst }
           if macroIsString then
             hs:=''''+hs+'''';
           current_scanner.substitutemacro(path,@hs[1],length(hs),
             current_scanner.line_no,current_scanner.inputfile.ref_index);
         end
        else
         begin
           hs:=FixFileName(hs);
           path:=ExtractFilePath(hs);
           name:=ExtractFileName(hs);
           { Special case for Delphi compatibility: '*' has to be replaced
             by the file name of the current source file.  }
           if (length(name)>=1) and
              (name[1]='*') then
             name:=ChangeFileExt(current_module.sourcefiles.get_file_name(current_filepos.fileindex),'')+ExtractFileExt(name);

           { try to find the file }
           found:=findincludefile(path,name,foundfile);
           if (not found) and (ExtractFileExt(name)='') then
            begin
              { try default extensions .inc , .pp and .pas }
              if (not found) then
               found:=findincludefile(path,ChangeFileExt(name,'.inc'),foundfile);
              if (not found) then
               found:=findincludefile(path,ChangeFileExt(name,sourceext),foundfile);
              if (not found) then
               found:=findincludefile(path,ChangeFileExt(name,pasext),foundfile);
            end;
           { if the name ends in dot, try without the dot }
           if (not found) and (ExtractFileExt(name)=ExtensionSeparator) and (Length(name)>=2) then
             found:=findincludefile(path,Copy(name,1,Length(name)-1),foundfile);
           if current_scanner.inputfilecount<max_include_nesting then
             begin
               inc(current_scanner.inputfilecount);
               { we need to reread the current char }
               dec(current_scanner.inputpointer);
               { reset c }
               c:=#0;
               { shutdown current file }
               current_scanner.tempcloseinputfile;
               { load new file }
               hp:=do_openinputfile(foundfile);
               hp.inc_path:=path;
               current_scanner.addfile(hp);
               current_module.sourcefiles.register_file(hp);
               if (not found) then
                Message1(scan_f_cannot_open_includefile,hs);
              if (not current_scanner.openinputfile) then
                Message1(scan_f_cannot_open_includefile,hs);
               Message1(scan_t_start_include_file,current_scanner.inputfile.path+current_scanner.inputfile.name);
               current_scanner.reload;
             end
           else
             Message(scan_f_include_deep_ten);
         end;
      end;


{*****************************************************************************
                            Preprocessor writing
*****************************************************************************}

{$ifdef PREPROCWRITE}
    constructor tpreprocfile.create(const fn:string);
      begin
        inherited create;
      { open outputfile }
        assign(f,fn);
        {$push}{$I-}
         rewrite(f);
        {$pop}
        if ioresult<>0 then
         Comment(V_Fatal,'can''t create file '+fn);
        getmem(buf,preprocbufsize);
        settextbuf(f,buf^,preprocbufsize);
      { reset }
        eolfound:=false;
        spacefound:=false;
      end;


    destructor tpreprocfile.destroy;
      begin
        close(f);
        freemem(buf,preprocbufsize);
      end;


    procedure tpreprocfile.add(const s:string);
      begin
        write(f,s);
      end;

    procedure tpreprocfile.addspace;
      begin
        if eolfound then
         begin
           writeln(f,'');
           eolfound:=false;
           spacefound:=false;
         end
        else
         if spacefound then
          begin
            write(f,' ');
            spacefound:=false;
          end;
      end;
{$endif PREPROCWRITE}


{*****************************************************************************
                              TPreProcStack
*****************************************************************************}

    constructor tpreprocstack.create(atyp : preproctyp;a:boolean;n:tpreprocstack);
      begin
        accept:=a;
        typ:=atyp;
        next:=n;
      end;

{*****************************************************************************
                              TReplayStack
*****************************************************************************}
    constructor treplaystack.Create(atoken:ttoken;aidtoken:ttoken;
      const aorgpattern,apattern:string;const acstringpattern:ansistring;
      apatternw:pcompilerwidestring;asettings:tsettings;
      atokenbuf:tdynamicarray;change_endian:boolean;anext:treplaystack);
      begin
        token:=atoken;
        idtoken:=aidtoken;
        orgpattern:=aorgpattern;
        pattern:=apattern;
        cstringpattern:=acstringpattern;
        initwidestring(patternw);
        if assigned(apatternw) then
          begin
            setlengthwidestring(patternw,apatternw^.len);
            move(apatternw^.data^,patternw^.data^,apatternw^.len*sizeof(tcompilerwidechar));
          end;
        settings:=asettings;
        tokenbuf:=atokenbuf;
        tokenbuf_needs_swapping:=change_endian;
        next:=anext;
      end;


    destructor treplaystack.destroy;
      begin
        donewidestring(patternw);
      end;

{*****************************************************************************
                              TDirectiveItem
*****************************************************************************}

    constructor TDirectiveItem.Create(AList:TFPHashObjectList;const n:string;p:tdirectiveproc);
      begin
        inherited Create(AList,n);
        is_conditional:=false;
        proc:=p;
      end;


    constructor TDirectiveItem.CreateCond(AList:TFPHashObjectList;const n:string;p:tdirectiveproc);
      begin
        inherited Create(AList,n);
        is_conditional:=true;
        proc:=p;
      end;

{****************************************************************************
                                TSCANNERFILE
 ****************************************************************************}

    constructor tscannerfile.create(const fn:string; is_macro: boolean = false);
      begin
        inputfile:=do_openinputfile(fn);
        if is_macro then
          inputfile.is_macro:=true;
        if assigned(current_module) then
          current_module.sourcefiles.register_file(inputfile);
      { reset localinput }
        c:=#0;
        inputbuffer:=nil;
        inputpointer:=nil;
        inputstart:=0;
      { reset scanner }
        preprocstack:=nil;
        replaystack:=nil;
        comment_level:=0;
        yylexcount:=0;
        block_type:=bt_general;
        line_no:=0;
        lastlinepos:=0;
        lasttokenpos:=0;
        nexttokenpos:=0;
        lasttoken:=NOTOKEN;
        nexttoken:=NOTOKEN;
        ignoredirectives:=TFPHashList.Create;
        change_endian_for_replay:=false;
      end;


    procedure tscannerfile.firstfile;
      begin
      { load block }
        if not openinputfile then
          Message1(scan_f_cannot_open_input,inputfile.name);
        reload;
      end;


    destructor tscannerfile.destroy;
      begin
        if assigned(current_module) and
           (current_module.state=ms_compiled) and
           (status.errorcount=0) then
          checkpreprocstack
        else
          begin
            while assigned(preprocstack) do
             poppreprocstack;
          end;
        while assigned(replaystack) do
          popreplaystack;
        if not inputfile.closed then
          closeinputfile;
        if inputfile.is_macro then
          inputfile.free;
        ignoredirectives.free;
      end;


    function tscannerfile.openinputfile:boolean;
      begin
        openinputfile:=inputfile.open;
      { load buffer }
        inputbuffer:=inputfile.buf;
        inputpointer:=inputfile.buf;
        inputstart:=inputfile.bufstart;
      { line }
        line_no:=0;
        lastlinepos:=0;
        lasttokenpos:=0;
        nexttokenpos:=0;
      end;


    procedure tscannerfile.closeinputfile;
      begin
        inputfile.close;
      { reset buffer }
        inputbuffer:=nil;
        inputpointer:=nil;
        inputstart:=0;
      { reset line }
        line_no:=0;
        lastlinepos:=0;
        lasttokenpos:=0;
        nexttokenpos:=0;
      end;


    function tscannerfile.tempopeninputfile:boolean;
      begin
        tempopeninputfile:=false;
        if inputfile.is_macro then
          exit;
        tempopeninputfile:=inputfile.tempopen;
      { reload buffer }
        inputbuffer:=inputfile.buf;
        inputpointer:=inputfile.buf;
        inputstart:=inputfile.bufstart;
      end;


    procedure tscannerfile.tempcloseinputfile;
      begin
        if inputfile.closed or inputfile.is_macro then
         exit;
        inputfile.setpos(inputstart+(inputpointer-inputbuffer));
        inputfile.tempclose;
      { reset buffer }
        inputbuffer:=nil;
        inputpointer:=nil;
        inputstart:=0;
      end;


    procedure tscannerfile.saveinputfile;
      begin
        inputfile.saveinputpointer:=inputpointer;
        inputfile.savelastlinepos:=lastlinepos;
        inputfile.saveline_no:=line_no;
      end;


    procedure tscannerfile.restoreinputfile;
      begin
        inputbuffer:=inputfile.buf;
        inputpointer:=inputfile.saveinputpointer;
        lastlinepos:=inputfile.savelastlinepos;
        line_no:=inputfile.saveline_no;
        if not inputfile.is_macro then
          parser_current_file:=inputfile.name;
      end;


    procedure tscannerfile.nextfile;
      var
        to_dispose : tinputfile;
      begin
        if assigned(inputfile.next) then
         begin
           if inputfile.is_macro then
             to_dispose:=inputfile
           else
             begin
               to_dispose:=nil;
               dec(inputfilecount);
             end;
           { we can allways close the file, no ? }
           inputfile.close;
           inputfile:=inputfile.next;
           if assigned(to_dispose) then
             to_dispose.free;
           restoreinputfile;
         end;
      end;


    procedure tscannerfile.startrecordtokens(buf:tdynamicarray);
      begin
        if not assigned(buf) then
          internalerror(200511172);
        if assigned(recordtokenbuf) then
          internalerror(200511173);
        recordtokenbuf:=buf;
        fillchar(last_settings,sizeof(last_settings),0);
        last_message:=nil;
        fillchar(last_filepos,sizeof(last_filepos),0);
      end;


    procedure tscannerfile.stoprecordtokens;
      begin
        if not assigned(recordtokenbuf) then
          internalerror(200511174);
        recordtokenbuf:=nil;
      end;

    function tscannerfile.is_recording_tokens: boolean;
      begin
        result:=assigned(recordtokenbuf);
      end;


    procedure tscannerfile.writetoken(t : ttoken);
      var
        b : byte;
      begin
        if ord(t)>$7f then
          begin
            b:=(ord(t) shr 8) or $80;
            recordtokenbuf.write(b,1);
          end;
        b:=ord(t) and $ff;
        recordtokenbuf.write(b,1);
      end;

    procedure tscannerfile.tokenwritesizeint(val : asizeint);
      begin
        recordtokenbuf.write(val,sizeof(asizeint));
      end;

    procedure tscannerfile.tokenwritelongint(val : longint);
      begin
        recordtokenbuf.write(val,sizeof(longint));
      end;

    procedure tscannerfile.tokenwriteshortint(val : shortint);
      begin
        recordtokenbuf.write(val,sizeof(shortint));
      end;

    procedure tscannerfile.tokenwriteword(val : word);
      begin
        recordtokenbuf.write(val,sizeof(word));
      end;

    procedure tscannerfile.tokenwritelongword(val : longword);
      begin
        recordtokenbuf.write(val,sizeof(longword));
      end;

    function tscannerfile.tokenreadsizeint : asizeint;
      var
        val : asizeint;
      begin
        replaytokenbuf.read(val,sizeof(asizeint));
        if change_endian_for_replay then
          val:=swapendian(val);
        result:=val;
      end;

    function tscannerfile.tokenreadlongword : longword;
      var
        val : longword;
      begin
        replaytokenbuf.read(val,sizeof(longword));
        if change_endian_for_replay then
          val:=swapendian(val);
        result:=val;
      end;

    function tscannerfile.tokenreadlongint : longint;
      var
        val : longint;
      begin
        replaytokenbuf.read(val,sizeof(longint));
        if change_endian_for_replay then
          val:=swapendian(val);
        result:=val;
      end;

    function tscannerfile.tokenreadshortint : shortint;
      var
        val : shortint;
      begin
        replaytokenbuf.read(val,sizeof(shortint));
        result:=val;
      end;

    function tscannerfile.tokenreadbyte : byte;
      var
        val : byte;
      begin
        replaytokenbuf.read(val,sizeof(byte));
        result:=val;
      end;

    function tscannerfile.tokenreadsmallint : smallint;
      var
        val : smallint;
      begin
        replaytokenbuf.read(val,sizeof(smallint));
        if change_endian_for_replay then
          val:=swapendian(val);
        result:=val;
      end;

    function tscannerfile.tokenreadword : word;
      var
        val : word;
      begin
        replaytokenbuf.read(val,sizeof(word));
        if change_endian_for_replay then
          val:=swapendian(val);
        result:=val;
      end;

   function tscannerfile.tokenreadenum(size : longint) : longword;
   begin
     if size=1 then
       result:=tokenreadbyte
     else if size=2 then
       result:=tokenreadword
     else if size=4 then
       result:=tokenreadlongword
     else
       internalerror(2013112901);
   end;

   procedure tscannerfile.tokenreadset(var b;size : longint);
   var
     i : longint;
   begin
     replaytokenbuf.read(b,size);
     if change_endian_for_replay then
       for i:=0 to size-1 do
         Pbyte(@b)[i]:=reverse_byte(Pbyte(@b)[i]);
   end;

   procedure tscannerfile.tokenwriteenum(var b;size : longint);
   begin
     recordtokenbuf.write(b,size);
   end;

   procedure tscannerfile.tokenwriteset(var b;size : longint);
   begin
     recordtokenbuf.write(b,size);
   end;


    procedure tscannerfile.tokenreadsettings(var asettings : tsettings; expected_size : asizeint);

    {    This procedure
       needs to be changed whenever
       globals.tsettings type is changed,
       the problem is that no error will appear
       before tests with generics are tested. PM }

       var
         startpos, endpos : longword;
      begin
        { WARNING all those fields need to be in the correct
        order otherwise cross_endian PPU reading will fail }
        startpos:=replaytokenbuf.pos;
        with asettings do
          begin
            alignment.procalign:=tokenreadlongint;
            alignment.loopalign:=tokenreadlongint;
            alignment.jumpalign:=tokenreadlongint;
            alignment.jumpalignskipmax:=tokenreadlongint;
            alignment.coalescealign:=tokenreadlongint;
            alignment.coalescealignskipmax:=tokenreadlongint;
            alignment.constalignmin:=tokenreadlongint;
            alignment.constalignmax:=tokenreadlongint;
            alignment.varalignmin:=tokenreadlongint;
            alignment.varalignmax:=tokenreadlongint;
            alignment.localalignmin:=tokenreadlongint;
            alignment.localalignmax:=tokenreadlongint;
            alignment.recordalignmin:=tokenreadlongint;
            alignment.recordalignmax:=tokenreadlongint;
            alignment.maxCrecordalign:=tokenreadlongint;
            tokenreadset(globalswitches,sizeof(globalswitches));
            tokenreadset(targetswitches,sizeof(targetswitches));
            tokenreadset(moduleswitches,sizeof(moduleswitches));
            tokenreadset(localswitches,sizeof(localswitches));
            tokenreadset(modeswitches,sizeof(modeswitches));
            tokenreadset(optimizerswitches,sizeof(optimizerswitches));
            tokenreadset(genwpoptimizerswitches,sizeof(genwpoptimizerswitches));
            tokenreadset(dowpoptimizerswitches,sizeof(dowpoptimizerswitches));
            tokenreadset(debugswitches,sizeof(debugswitches));
            { 0: old behaviour for sets <=256 elements
              >0: round to this size }
            setalloc:=tokenreadshortint;
            packenum:=tokenreadshortint;

            packrecords:=tokenreadshortint;
            maxfpuregisters:=tokenreadshortint;


            cputype:=tcputype(tokenreadenum(sizeof(tcputype)));
            optimizecputype:=tcputype(tokenreadenum(sizeof(tcputype)));
            fputype:=tfputype(tokenreadenum(sizeof(tfputype)));
            asmmode:=tasmmode(tokenreadenum(sizeof(tasmmode)));
            interfacetype:=tinterfacetypes(tokenreadenum(sizeof(tinterfacetypes)));
            defproccall:=tproccalloption(tokenreadenum(sizeof(tproccalloption)));
            { tstringencoding is word type,
              thus this should be OK here }
            sourcecodepage:=tstringEncoding(tokenreadword);

            minfpconstprec:=tfloattype(tokenreadenum(sizeof(tfloattype)));

            disabledircache:=boolean(tokenreadbyte);

            tlsmodel:=ttlsmodel(tokenreadenum(sizeof(ttlsmodel)));
{ TH: Since the field was conditional originally, it was not stored in PPUs.  }
{ While adding ControllerSupport constant, I decided not to store ct_none     }
{ on targets not supporting controllers, but this might be changed here and   }
{ in tokenwritesettings in the future to unify the PPU structure and handling }
{ of this field in the compiler.                                              }
{$PUSH}
 {$WARN 6018 OFF} (* Unreachable code due to compile time evaluation *)
            if ControllerSupport then
             controllertype:=tcontrollertype(tokenreadenum(sizeof(tcontrollertype)))
            else
             ControllerType:=ct_none;
{$POP}
           endpos:=replaytokenbuf.pos;
           if endpos-startpos<>expected_size then
             Comment(V_Error,'Wrong size of Settings read-in');
         end;
     end;

    procedure tscannerfile.tokenwritesettings(var asettings : tsettings; var size : asizeint);

    {    This procedure
       needs to be changed whenever
       globals.tsettings type is changed,
       the problem is that no error will appear
       before tests with generics are tested. PM }

       var
         sizepos, startpos, endpos : longword;
      begin
        { WARNING all those fields need to be in the correct
        order otherwise cross_endian PPU reading will fail }
        sizepos:=recordtokenbuf.pos;
        size:=0;
        tokenwritesizeint(size);
        startpos:=recordtokenbuf.pos;
        with asettings do
          begin
            tokenwritelongint(alignment.procalign);
            tokenwritelongint(alignment.loopalign);
            tokenwritelongint(alignment.jumpalign);
            tokenwritelongint(alignment.jumpalignskipmax);
            tokenwritelongint(alignment.coalescealign);
            tokenwritelongint(alignment.coalescealignskipmax);
            tokenwritelongint(alignment.constalignmin);
            tokenwritelongint(alignment.constalignmax);
            tokenwritelongint(alignment.varalignmin);
            tokenwritelongint(alignment.varalignmax);
            tokenwritelongint(alignment.localalignmin);
            tokenwritelongint(alignment.localalignmax);
            tokenwritelongint(alignment.recordalignmin);
            tokenwritelongint(alignment.recordalignmax);
            tokenwritelongint(alignment.maxCrecordalign);
            tokenwriteset(globalswitches,sizeof(globalswitches));
            tokenwriteset(targetswitches,sizeof(targetswitches));
            tokenwriteset(moduleswitches,sizeof(moduleswitches));
            tokenwriteset(localswitches,sizeof(localswitches));
            tokenwriteset(modeswitches,sizeof(modeswitches));
            tokenwriteset(optimizerswitches,sizeof(optimizerswitches));
            tokenwriteset(genwpoptimizerswitches,sizeof(genwpoptimizerswitches));
            tokenwriteset(dowpoptimizerswitches,sizeof(dowpoptimizerswitches));
            tokenwriteset(debugswitches,sizeof(debugswitches));
            { 0: old behaviour for sets <=256 elements
              >0: round to this size }
            tokenwriteshortint(setalloc);
            tokenwriteshortint(packenum);
            tokenwriteshortint(packrecords);
            tokenwriteshortint(maxfpuregisters);

            tokenwriteenum(cputype,sizeof(tcputype));
            tokenwriteenum(optimizecputype,sizeof(tcputype));
            tokenwriteenum(fputype,sizeof(tfputype));
            tokenwriteenum(asmmode,sizeof(tasmmode));
            tokenwriteenum(interfacetype,sizeof(tinterfacetypes));
            tokenwriteenum(defproccall,sizeof(tproccalloption));
            { tstringencoding is word type,
              thus this should be OK here }
            tokenwriteword(sourcecodepage);

            tokenwriteenum(minfpconstprec,sizeof(tfloattype));

            recordtokenbuf.write(byte(disabledircache),1);

            tokenwriteenum(tlsmodel,sizeof(tlsmodel));

{ TH: See note about controllertype field in tokenreadsettings. }
{$PUSH}
 {$WARN 6018 OFF} (* Unreachable code due to compile time evaluation *)
            if ControllerSupport then
              tokenwriteenum(controllertype,sizeof(tcontrollertype));
{$POP}
           endpos:=recordtokenbuf.pos;
           size:=endpos-startpos;
           recordtokenbuf.seek(sizepos);
           tokenwritesizeint(size);
           recordtokenbuf.seek(endpos);
         end;
     end;


    procedure tscannerfile.recordtoken;
      var
        t : ttoken;
        s : tspecialgenerictoken;
        len,msgnb,copy_size : asizeint;
        val : longint;
        b : byte;
        pmsg : pmessagestaterecord;
      begin
        if not assigned(recordtokenbuf) then
          internalerror(200511176);
        t:=_GENERICSPECIALTOKEN;
        { settings changed? }
        { last field pmessage is handled separately below in
          ST_LOADMESSAGES }
        if CompareByte(current_settings,last_settings,
             sizeof(current_settings)-sizeof(pointer))<>0 then
          begin
            { use a special token to record it }
            s:=ST_LOADSETTINGS;
            writetoken(t);
            recordtokenbuf.write(s,1);
            copy_size:=sizeof(current_settings)-sizeof(pointer);
            tokenwritesettings(current_settings,copy_size);
            last_settings:=current_settings;
          end;

        if current_settings.pmessage<>last_message then
          begin
            { use a special token to record it }
            s:=ST_LOADMESSAGES;
            writetoken(t);
            recordtokenbuf.write(s,1);
            msgnb:=0;
            pmsg:=current_settings.pmessage;
            while assigned(pmsg) do
              begin
                if msgnb=high(asizeint) then
                  { Too many messages }
                  internalerror(2011090401);
                inc(msgnb);
                pmsg:=pmsg^.next;
              end;
            tokenwritesizeint(msgnb);
            pmsg:=current_settings.pmessage;
            while assigned(pmsg) do
              begin
                { What about endianess here?}
                { SB: this is handled by tokenreadlongint }
                val:=pmsg^.value;
                tokenwritelongint(val);
                val:=ord(pmsg^.state);
                tokenwritelongint(val);
                pmsg:=pmsg^.next;
              end;
            last_message:=current_settings.pmessage;
          end;

        { file pos changes? }
        if current_tokenpos.fileindex<>last_filepos.fileindex then
          begin
            s:=ST_FILEINDEX;
            writetoken(t);
            recordtokenbuf.write(s,1);
            tokenwriteword(current_tokenpos.fileindex);
            last_filepos.fileindex:=current_tokenpos.fileindex;
            last_filepos.line:=0;
          end;
        if current_tokenpos.line<>last_filepos.line then
          begin
            s:=ST_LINE;
            writetoken(t);
            recordtokenbuf.write(s,1);
            tokenwritelongint(current_tokenpos.line);
            last_filepos.line:=current_tokenpos.line;
            last_filepos.column:=0;
          end;
        if current_tokenpos.column<>last_filepos.column then
          begin
            s:=ST_COLUMN;
            writetoken(t);
            { can the column be written packed? }
            if current_tokenpos.column<$80 then
              begin
                b:=$80 or current_tokenpos.column;
                recordtokenbuf.write(b,1);
              end
            else
              begin
                recordtokenbuf.write(s,1);
                tokenwriteword(current_tokenpos.column);
              end;
            last_filepos.column:=current_tokenpos.column;
          end;

        writetoken(token);
        if token<>_GENERICSPECIALTOKEN then
          writetoken(idtoken);
        case token of
          _CWCHAR,
          _CWSTRING :
            begin
              tokenwritesizeint(patternw^.len);
              if patternw^.len>0 then
                recordtokenbuf.write(patternw^.data^,patternw^.len*sizeof(tcompilerwidechar));
            end;
          _CSTRING:
            begin
              len:=length(cstringpattern);
              tokenwritesizeint(len);
              if len>0 then
                recordtokenbuf.write(cstringpattern[1],len);
            end;
          _CCHAR,
          _INTCONST,
          _REALNUMBER :
            begin
              { pexpr.pas messes with pattern in case of negative integer consts,
                see around line 2562 the comment of JM; remove the - before recording it
                                                     (FK)
              }
              if (token=_INTCONST) and (pattern[1]='-') then
                delete(pattern,1,1);
              recordtokenbuf.write(pattern[0],1);
              recordtokenbuf.write(pattern[1],length(pattern));
            end;
          _ID :
            begin
              recordtokenbuf.write(orgpattern[0],1);
              recordtokenbuf.write(orgpattern[1],length(orgpattern));
            end;
          else
            ;
        end;
      end;


    procedure tscannerfile.startreplaytokens(buf:tdynamicarray; change_endian:boolean);
      begin
        if not assigned(buf) then
          internalerror(200511175);

        { save current scanner state }
        replaystack:=treplaystack.create(token,idtoken,orgpattern,pattern,
          cstringpattern,patternw,current_settings,replaytokenbuf,change_endian_for_replay,replaystack);
        if assigned(inputpointer) then
          dec(inputpointer);
        { install buffer }
        replaytokenbuf:=buf;

        { Initialize value of change_endian_for_replay variable }
        change_endian_for_replay:=change_endian;

        { reload next token }
        replaytokenbuf.seek(0);
        replaytoken;
      end;


    function tscannerfile.readtoken: ttoken;
      var
        b,b2 : byte;
      begin
        replaytokenbuf.read(b,1);
        if (b and $80)<>0 then
          begin
            replaytokenbuf.read(b2,1);
            result:=ttoken(((b and $7f) shl 8) or b2);
          end
        else
          result:=ttoken(b);
      end;


    procedure tscannerfile.replaytoken;
      var
        wlen,mesgnb,copy_size : asizeint;
        specialtoken : tspecialgenerictoken;
        i : byte;
        pmsg,prevmsg : pmessagestaterecord;
      begin
        if not assigned(replaytokenbuf) then
          internalerror(200511177);
        { End of replay buffer? Then load the next char from the file again }
        if replaytokenbuf.pos>=replaytokenbuf.size then
          begin
            token:=replaystack.token;
            idtoken:=replaystack.idtoken;
            pattern:=replaystack.pattern;
            orgpattern:=replaystack.orgpattern;
            setlengthwidestring(patternw,replaystack.patternw^.len);
            move(replaystack.patternw^.data^,patternw^.data^,replaystack.patternw^.len*sizeof(tcompilerwidechar));
            cstringpattern:=replaystack.cstringpattern;
            replaytokenbuf:=replaystack.tokenbuf;
            change_endian_for_replay:=replaystack.tokenbuf_needs_swapping;
            { restore compiler settings }
            current_settings:=replaystack.settings;
            popreplaystack;
            if assigned(inputpointer) then
              begin
                c:=inputpointer^;
                inc(inputpointer);
              end;
            exit;
          end;
        repeat
          { load token from the buffer }
          token:=readtoken;
          if token<>_GENERICSPECIALTOKEN then
            idtoken:=readtoken
          else
            idtoken:=_NOID;
          case token of
            _CWCHAR,
            _CWSTRING :
              begin
                wlen:=tokenreadsizeint;
                setlengthwidestring(patternw,wlen);
                if wlen>0 then
                  replaytokenbuf.read(patternw^.data^,patternw^.len*sizeof(tcompilerwidechar));
                orgpattern:='';
                pattern:='';
                cstringpattern:='';
              end;
            _CSTRING:
              begin
                wlen:=tokenreadsizeint;
                if wlen>0 then
                  begin
                    setlength(cstringpattern,wlen);
                    replaytokenbuf.read(cstringpattern[1],wlen);
                  end
                else
                  cstringpattern:='';
                orgpattern:='';
                pattern:='';
              end;
            _CCHAR,
            _INTCONST,
            _REALNUMBER :
              begin
                replaytokenbuf.read(pattern[0],1);
                replaytokenbuf.read(pattern[1],length(pattern));
                orgpattern:='';
              end;
            _ID :
              begin
                replaytokenbuf.read(orgpattern[0],1);
                replaytokenbuf.read(orgpattern[1],length(orgpattern));
                pattern:=upper(orgpattern);
              end;
            _GENERICSPECIALTOKEN:
              begin
                replaytokenbuf.read(specialtoken,1);
                { packed column? }
                if (ord(specialtoken) and $80)<>0 then
                  begin
                      current_tokenpos.column:=ord(specialtoken) and $7f;
                      current_filepos:=current_tokenpos;
                  end
                else
                  case specialtoken of
                    ST_LOADSETTINGS:
                      begin
                        copy_size:=tokenreadsizeint;
                        //if copy_size <> sizeof(current_settings)-sizeof(pointer) then
                        //  internalerror(2011090501);
                        {
                        replaytokenbuf.read(current_settings,copy_size);
                        }
                        tokenreadsettings(current_settings,copy_size);
                      end;
                    ST_LOADMESSAGES:
                      begin
                        current_settings.pmessage:=nil;
                        mesgnb:=tokenreadsizeint;
                        prevmsg:=nil;
                        for i:=1 to mesgnb do
                          begin
                            new(pmsg);
                            if i=1 then
                              current_settings.pmessage:=pmsg
                            else
                              prevmsg^.next:=pmsg;
                            pmsg^.value:=tokenreadlongint;
                            pmsg^.state:=tmsgstate(tokenreadlongint);
                            pmsg^.next:=nil;
                            prevmsg:=pmsg;
                          end;
                      end;
                    ST_LINE:
                      begin
                        current_tokenpos.line:=tokenreadlongint;
                        current_filepos:=current_tokenpos;
                      end;
                    ST_COLUMN:
                      begin
                        current_tokenpos.column:=tokenreadword;
                        current_filepos:=current_tokenpos;
                      end;
                    ST_FILEINDEX:
                      begin
                        current_tokenpos.fileindex:=tokenreadword;
                        current_filepos:=current_tokenpos;
                      end;
                  end;
                continue;
              end;
            else
              ;
          end;
          break;
        until false;
      end;


    procedure tscannerfile.addfile(hp:tinputfile);
      begin
        saveinputfile;
        { add to list }
        hp.next:=inputfile;
        inputfile:=hp;
        { load new inputfile }
        restoreinputfile;
      end;


    procedure tscannerfile.reload;
      begin
        with inputfile do
         begin
           { when nothing more to read then leave immediatly, so we
             don't change the current_filepos and leave it point to the last
             char }
           if (c=#26) and (not assigned(next)) then
            exit;
           repeat
           { still more to read?, then change the #0 to a space so its seen
             as a seperator, this can't be used for macro's which can change
             the place of the #0 in the buffer with tempopen }
             if (c=#0) and (bufsize>0) and
                not(inputfile.is_macro) and
                (inputpointer-inputbuffer<bufsize) then
              begin
                c:=' ';
                inc(inputpointer);
                exit;
              end;
           { can we read more from this file ? }
             if (c<>#26) and (not endoffile) then
              begin
                readbuf;
                inputpointer:=buf;
                inputbuffer:=buf;
                inputstart:=bufstart;
              { first line? }
                if line_no=0 then
                 begin
                   c:=inputpointer^;
                   { eat utf-8 signature? }
                   if (ord(inputpointer^)=$ef) and
                     (ord((inputpointer+1)^)=$bb) and
                     (ord((inputpointer+2)^)=$bf) then
                     begin
                       (* we don't support including files with an UTF-8 bom
                          inside another file that wasn't encoded as UTF-8
                          already (we don't support {$codepage xxx} switches in
                          the middle of a file either) *)
                       if (current_settings.sourcecodepage<>CP_UTF8) and
                          not current_module.in_global then
                         Message(scanner_f_illegal_utf8_bom);
                       inc(inputpointer,3);
                       message(scan_c_switching_to_utf8);
                       current_settings.sourcecodepage:=CP_UTF8;
                       exclude(current_settings.moduleswitches,cs_system_codepage);
                       include(current_settings.moduleswitches,cs_explicit_codepage);
                     end;

                   line_no:=1;
                   if cs_asm_source in current_settings.globalswitches then
                     inputfile.setline(line_no,inputstart+inputpointer-inputbuffer);
                 end;
              end
             else
              begin
              { load eof position in tokenpos/current_filepos }
                gettokenpos;
              { close file }
                closeinputfile;
              { no next module, than EOF }
                if not assigned(inputfile.next) then
                 begin
                   c:=#26;
                   exit;
                 end;
              { load next file and reopen it }
                nextfile;
                tempopeninputfile;
              { status }
                Message1(scan_t_back_in,inputfile.name);
              end;
           { load next char }
             c:=inputpointer^;
             inc(inputpointer);
           until c<>#0; { if also end, then reload again }
         end;
      end;


    procedure tscannerfile.substitutemacro(const macname:string;p:pchar;len,line,fileindex:longint);
      var
        hp : tinputfile;
      begin
        { save old postion }
        dec(inputpointer);
        tempcloseinputfile;
      { create macro 'file' }
        { use special name to dispose after !! }
        hp:=do_openinputfile('_Macro_.'+macname);
        addfile(hp);
        with inputfile do
         begin
           setmacro(p,len);
         { local buffer }
           inputbuffer:=buf;
           inputpointer:=buf;
           inputstart:=bufstart;
           ref_index:=fileindex;
         end;
      { reset line }
        line_no:=line;
        lastlinepos:=0;
        lasttokenpos:=0;
        nexttokenpos:=0;
      { load new c }
        c:=inputpointer^;
        inc(inputpointer);
      end;


    procedure tscannerfile.do_gettokenpos(out tokenpos: longint; out filepos: tfileposinfo);
      begin
        tokenpos:=inputstart+(inputpointer-inputbuffer);
        filepos.line:=line_no;
        filepos.column:=tokenpos-lastlinepos;
        filepos.fileindex:=inputfile.ref_index;
        filepos.moduleindex:=current_module.unit_index;
      end;


    procedure tscannerfile.gettokenpos;
    { load the values of tokenpos and lasttokenpos }
      begin
        do_gettokenpos(lasttokenpos,current_tokenpos);
        current_filepos:=current_tokenpos;
      end;


    procedure tscannerfile.cachenexttokenpos;
      begin
        do_gettokenpos(nexttokenpos,next_filepos);
      end;


    procedure tscannerfile.setnexttoken;
      begin
        token:=nexttoken;
        nexttoken:=NOTOKEN;
        lasttokenpos:=nexttokenpos;
        current_tokenpos:=next_filepos;
        current_filepos:=current_tokenpos;
        nexttokenpos:=0;
      end;


    procedure tscannerfile.savetokenpos;
      begin
        oldlasttokenpos:=lasttokenpos;
        oldcurrent_filepos:=current_filepos;
        oldcurrent_tokenpos:=current_tokenpos;
      end;


    procedure tscannerfile.restoretokenpos;
      begin
        lasttokenpos:=oldlasttokenpos;
        current_filepos:=oldcurrent_filepos;
        current_tokenpos:=oldcurrent_tokenpos;
      end;


    procedure tscannerfile.inc_comment_level;
      begin
         if (m_nested_comment in current_settings.modeswitches) then
           inc(comment_level)
         else
           comment_level:=1;
         if (comment_level>1) then
          begin
             savetokenpos;
             gettokenpos; { update for warning }
             Message1(scan_w_comment_level,tostr(comment_level));
             restoretokenpos;
          end;
      end;


    procedure tscannerfile.dec_comment_level;
      begin
         if (m_nested_comment in current_settings.modeswitches) then
           dec(comment_level)
         else
           comment_level:=0;
      end;


    procedure tscannerfile.linebreak;
      var
         cur : char;
      begin
        with inputfile do
         begin
           if (byte(inputpointer^)=0) and not(endoffile) then
             begin
               cur:=c;
               reload;
               if byte(cur)+byte(c)<>23 then
                 dec(inputpointer);
             end
           else
             begin
               { Support all combination of #10 and #13 as line break }
               if (byte(inputpointer^)+byte(c)=23) then
                 inc(inputpointer);
             end;
           { Always return #10 as line break }
           c:=#10;
           { increase line counters }
           lastlinepos:=inputstart+(inputpointer-inputbuffer);
           inc(line_no);
           { update linebuffer }
           if cs_asm_source in current_settings.globalswitches then
             inputfile.setline(line_no,lastlinepos);
           { update for status and call the show status routine,
             but don't touch current_filepos ! }
           savetokenpos;
           gettokenpos; { update for v_status }
           inc(status.compiledlines);
           ShowStatus;
           restoretokenpos;
         end;
      end;


    procedure tscannerfile.illegal_char(c:char);
      var
        s : string;
      begin
        if c in [#32..#255] then
          s:=''''+c+''''
        else
          s:='#'+tostr(ord(c));
        Message2(scan_f_illegal_char,s,'$'+hexstr(ord(c),2));
      end;


    procedure tscannerfile.end_of_file;
      begin
        checkpreprocstack;
        Message(scan_f_end_of_file);
      end;

  {-------------------------------------------
           IF Conditional Handling
  -------------------------------------------}

    procedure tscannerfile.checkpreprocstack;
      begin
      { check for missing ifdefs }
        while assigned(preprocstack) do
         begin
           Message4(scan_e_endif_expected,preprocstring[preprocstack.typ],preprocstack.name,
             current_module.sourcefiles.get_file_name(preprocstack.fileindex),
             tostr(preprocstack.line_nb));
           poppreprocstack;
         end;
      end;


    procedure tscannerfile.poppreprocstack;
      var
        hp : tpreprocstack;
      begin
        if assigned(preprocstack) then
         begin
           Message1(scan_c_endif_found,preprocstack.name);
           hp:=preprocstack.next;
           preprocstack.free;
           preprocstack:=hp;
         end
        else
         Message(scan_e_endif_without_if);
      end;


    procedure tscannerfile.ifpreprocstack(atyp:preproctyp;compile_time_predicate:tcompile_time_predicate;messid:longint);
      var
        condition: Boolean;
        valuedescr: String;
      begin
        if (preprocstack=nil) or preprocstack.accept then
          condition:=compile_time_predicate(valuedescr)
        else
          begin
            condition:= false;
            valuedescr:= '';
          end;
        preprocstack:=tpreprocstack.create(atyp, condition, preprocstack);
        preprocstack.name:=valuedescr;
        preprocstack.line_nb:=line_no;
        preprocstack.fileindex:=current_filepos.fileindex;
        if preprocstack.accept then
          Message2(messid,preprocstack.name,'accepted')
        else
          Message2(messid,preprocstack.name,'rejected');
      end;

    procedure tscannerfile.elsepreprocstack;
      begin
        if assigned(preprocstack) and
           (preprocstack.typ<>pp_else) then
         begin
           if (preprocstack.typ=pp_elseif) then
             preprocstack.accept:=false
           else
             if (not(assigned(preprocstack.next)) or (preprocstack.next.accept)) then
               preprocstack.accept:=not preprocstack.accept;
           preprocstack.typ:=pp_else;
           preprocstack.line_nb:=line_no;
           preprocstack.fileindex:=current_filepos.fileindex;
           if preprocstack.accept then
            Message2(scan_c_else_found,preprocstack.name,'accepted')
           else
            Message2(scan_c_else_found,preprocstack.name,'rejected');
         end
        else
         Message(scan_e_endif_without_if);
      end;

    procedure tscannerfile.elseifpreprocstack(compile_time_predicate:tcompile_time_predicate);
      var
        valuedescr: String;
      begin
        if assigned(preprocstack) and
           (preprocstack.typ in [pp_if,pp_ifdef,pp_ifndef,pp_elseif]) then
         begin
           { when the branch is accepted we use pp_elseif so we know that
             all the next branches need to be rejected. when this branch is still
             not accepted then leave it at pp_if }
           if (preprocstack.typ=pp_elseif) then
             preprocstack.accept:=false
           else if (preprocstack.typ in [pp_if,pp_ifdef,pp_ifndef]) and preprocstack.accept then
               begin
                 preprocstack.accept:=false;
                 preprocstack.typ:=pp_elseif;
               end
           else if (not(assigned(preprocstack.next)) or (preprocstack.next.accept))
                   and compile_time_predicate(valuedescr) then
               begin
                 preprocstack.name:=valuedescr;
                 preprocstack.accept:=true;
                 preprocstack.typ:=pp_elseif;
               end;

           preprocstack.line_nb:=line_no;
           preprocstack.fileindex:=current_filepos.fileindex;
           if preprocstack.accept then
             Message2(scan_c_else_found,preprocstack.name,'accepted')
           else
             Message2(scan_c_else_found,preprocstack.name,'rejected');
         end
        else
         Message(scan_e_endif_without_if);
      end;


    procedure tscannerfile.popreplaystack;
      var
        hp : treplaystack;
      begin
        if assigned(replaystack) then
         begin
           hp:=replaystack.next;
           replaystack.free;
           replaystack:=hp;
         end;
      end;


    function tscannerfile.replay_stack_depth:longint;
      var
        tmp: treplaystack;
      begin
        result:=0;
        tmp:=replaystack;
        while assigned(tmp) do
          begin
            inc(result);
            tmp:=tmp.next;
          end;
      end;

    procedure tscannerfile.handleconditional(p:tdirectiveitem);
      begin
        savetokenpos;
        repeat
          current_scanner.gettokenpos;
          Message1(scan_d_handling_switch,'$'+p.name);
          p.proc();
          { accept the text ? }
          if (current_scanner.preprocstack=nil) or current_scanner.preprocstack.accept then
           break
          else
           begin
             current_scanner.gettokenpos;
             Message(scan_c_skipping_until);
             repeat
               current_scanner.skipuntildirective;
               if not (m_mac in current_settings.modeswitches) then
                 p:=tdirectiveitem(turbo_scannerdirectives.Find(current_scanner.readid))
               else
                 p:=tdirectiveitem(mac_scannerdirectives.Find(current_scanner.readid));
             until assigned(p) and (p.is_conditional);
             current_scanner.gettokenpos;
           end;
        until false;
        restoretokenpos;
      end;


    procedure tscannerfile.handledirectives;
      var
         t  : tdirectiveitem;
         hs : string;
      begin
         gettokenpos;
         readchar; {Remove the $}
         hs:=readid;
         { handle empty directive }
         if hs='' then
           begin
             Message1(scan_w_illegal_switch,'$');
             exit;
           end;
{$ifdef PREPROCWRITE}
         if parapreprocess then
          begin
            if not (m_mac in current_settings.modeswitches) then
              t:=tdirectiveitem(turbo_scannerdirectives.Find(hs))
            else
              t:=tdirectiveitem(mac_scannerdirectives.Find(hs));
            if assigned(t) and not(t.is_conditional) then
             begin
               preprocfile.AddSpace;
               preprocfile.Add('{$'+hs+current_scanner.readcomment+'}');
               exit;
             end;
          end;
{$endif PREPROCWRITE}
         { skip this directive? }
         if (ignoredirectives.find(hs)<>nil) then
          begin
            if (comment_level>0) then
             readcomment;
            { we've read the whole comment }
            current_commentstyle:=comment_none;
            exit;
          end;
         { Check for compiler switches }
         while (length(hs)=1) and (c in ['-','+']) do
          begin
            Message1(scan_d_handling_switch,'$'+hs+c);
            HandleSwitch(hs[1],c);
            current_scanner.readchar; {Remove + or -}
            if c=',' then
             begin
               current_scanner.readchar;   {Remove , }
               { read next switch, support $v+,$+}
               hs:=current_scanner.readid;
               if (hs='') then
                begin
                  if (c='$') and (m_fpc in current_settings.modeswitches) then
                   begin
                     current_scanner.readchar;  { skip $ }
                     hs:=current_scanner.readid;
                   end;
                  if (hs='') then
                   Message1(scan_w_illegal_directive,'$'+c);
                end;
             end
            else
             hs:='';
          end;
         { directives may follow switches after a , }
         if hs<>'' then
          begin
            if not (m_mac in current_settings.modeswitches) then
              t:=tdirectiveitem(turbo_scannerdirectives.Find(hs))
            else
              t:=tdirectiveitem(mac_scannerdirectives.Find(hs));

            if assigned(t) then
             begin
               if t.is_conditional then
                handleconditional(t)
               else
                begin
                  Message1(scan_d_handling_switch,'$'+hs);
                  t.proc();
                end;
             end
            else
             begin
               current_scanner.ignoredirectives.Add(hs,nil);
               Message1(scan_w_illegal_directive,'$'+hs);
             end;
            { conditionals already read the comment }
            if (current_scanner.comment_level>0) then
             current_scanner.readcomment;
            { we've read the whole comment }
            current_commentstyle:=comment_none;
          end;
      end;


    procedure tscannerfile.readchar;
      begin
        c:=inputpointer^;
        if c=#0 then
          reload
        else
          inc(inputpointer);
      end;


    procedure tscannerfile.readstring;
      var
        i : longint;
        err : boolean;
      begin
        err:=false;
        i:=0;
        repeat
          case c of
            '_',
            '0'..'9',
            'A'..'Z' :
              begin
                if i<255 then
                 begin
                   inc(i);
                   orgpattern[i]:=c;
                   pattern[i]:=c;
                 end
                else
                 begin
                   if not err then
                     begin
                       Message(scan_e_string_exceeds_255_chars);
                       err:=true;
                     end;
                 end;
                c:=inputpointer^;
                inc(inputpointer);
              end;
            'a'..'z' :
              begin
                if i<255 then
                 begin
                   inc(i);
                   orgpattern[i]:=c;
                   pattern[i]:=chr(ord(c)-32)
                 end
                else
                 begin
                   if not err then
                     begin
                       Message(scan_e_string_exceeds_255_chars);
                       err:=true;
                     end;
                 end;
                c:=inputpointer^;
                inc(inputpointer);
              end;
            #0 :
              reload;
            else
              break;
          end;
        until false;
        orgpattern[0]:=chr(i);
        pattern[0]:=chr(i);
      end;


    procedure tscannerfile.readnumber;
      var
        base,
        i  : longint;
      begin
        case c of
          '%' :
            begin
              readchar;
              base:=2;
              pattern[1]:='%';
              i:=1;
            end;
          '&' :
            begin
              readchar;
              base:=8;
              pattern[1]:='&';
              i:=1;
            end;
          '$' :
            begin
              readchar;
              base:=16;
              pattern[1]:='$';
              i:=1;
            end;
          else
            begin
              base:=10;
              i:=0;
            end;
        end;
        while ((base>=10) and (c in ['0'..'9'])) or
              ((base=16) and (c in ['A'..'F','a'..'f'])) or
              ((base=8) and (c in ['0'..'7'])) or
              ((base=2) and (c in ['0'..'1'])) do
         begin
           if i<255 then
            begin
              inc(i);
              pattern[i]:=c;
            end;
           readchar;
         end;
        pattern[0]:=chr(i);
      end;


    function tscannerfile.readid:string;
      begin
        readstring;
        readid:=pattern;
      end;


    function tscannerfile.readval:longint;
      var
        l : longint;
        w : integer;
      begin
        readnumber;
        val(pattern,l,w);
        readval:=l;
      end;


    function tscannerfile.readcomment(include_special_char: boolean):string;
      var
        i : longint;
      begin
        i:=0;
        repeat
          case c of
            '{' :
              begin
                if (include_special_char) and (i<255) then
                begin
                  inc(i);
                  readcomment[i]:=c;
                end;

                if current_commentstyle=comment_tp then
                  inc_comment_level;
              end;
            '}' :
              begin
                if (include_special_char) and (i<255) then
                begin
                  inc(i);
                  readcomment[i]:=c;
                end;

                if current_commentstyle=comment_tp then
                  begin
                    readchar;
                    dec_comment_level;


                    if comment_level=0 then
                      break
                    else
                      continue;
                  end;
              end;
            '*' :
              begin
                if current_commentstyle=comment_oldtp then
                  begin
                    readchar;
                    if c=')' then
                      begin
                        readchar;
                        dec_comment_level;
                        break;
                      end
                    else
                    { Add both characters !!}
                      if (i<255) then
                        begin
                          inc(i);
                          readcomment[i]:='*';
                          if (i<255) then
                            begin
                              inc(i);
                              readcomment[i]:=c;
                            end;
                        end;
                  end
                else
                { Not old TP comment, so add...}
                  begin
                    if (i<255) then
                      begin
                        inc(i);
                        readcomment[i]:='*';
                      end;
                  end;
              end;
            #10,#13 :
              linebreak;
            #26 :
              end_of_file;
            else
              begin
                if (i<255) then
                  begin
                    inc(i);
                    readcomment[i]:=c;
                  end;
              end;
          end;
          readchar;
        until false;
        readcomment[0]:=chr(i);
      end;


    function tscannerfile.readquotedstring:string;
      var
        i : longint;
        msgwritten : boolean;
      begin
        i:=0;
        msgwritten:=false;
        if (c='''') then
          begin
            repeat
              readchar;
              case c of
                #26 :
                  end_of_file;
                #10,#13 :
                  Message(scan_f_string_exceeds_line);
                '''' :
                  begin
                    readchar;
                    if c<>'''' then
                     break;
                  end;
              end;
              if i<255 then
                begin
                  inc(i);
                  result[i]:=c;
                end
              else
                begin
                  if not msgwritten then
                    begin
                      Message(scan_e_string_exceeds_255_chars);
                      msgwritten:=true;
                    end;
                 end;
            until false;
          end;
        result[0]:=chr(i);
      end;


    function tscannerfile.readstate:char;
      var
        state : char;
      begin
        state:=' ';
        if c=' ' then
         begin
           current_scanner.skipspace;
           current_scanner.readid;
           if pattern='ON' then
            state:='+'
           else
            if pattern='OFF' then
             state:='-';
         end
        else
         state:=c;
        if not (state in ['+','-']) then
         Message(scan_e_wrong_switch_toggle);
        readstate:=state;
      end;


    function tscannerfile.readoptionalstate(fallback:char):char;
      var
        state : char;
      begin
        state:=' ';
        if c=' ' then
         begin
           current_scanner.skipspace;
           if c in ['*','}'] then
             state:=fallback
           else
             begin
               current_scanner.readid;
               if pattern='ON' then
                state:='+'
               else
                if pattern='OFF' then
                 state:='-';
             end;
         end
        else
          if c in ['*','}'] then
            state:=fallback
          else
            state:=c;
        if not (state in ['+','-']) then
         Message(scan_e_wrong_switch_toggle);
        readoptionalstate:=state;
      end;


    function tscannerfile.readstatedefault:char;
      var
        state : char;
      begin
        state:=' ';
        if c=' ' then
         begin
           current_scanner.skipspace;
           current_scanner.readid;
           if pattern='ON' then
            state:='+'
           else
            if pattern='OFF' then
             state:='-'
            else
             if pattern='DEFAULT' then
              state:='*';
         end
        else
         state:=c;
        if not (state in ['+','-','*']) then
         Message(scan_e_wrong_switch_toggle_default);
        readstatedefault:=state;
      end;


    procedure tscannerfile.skipspace;
      begin
        repeat
          case c of
            #26 :
              begin
                reload;
                if (c=#26) and not assigned(inputfile.next) then
                  break;
                continue;
              end;
            #10,
            #13 :
              linebreak;
            #9,#11,#12,' ' :
              ;
            else
              break;
          end;
          readchar;
        until false;
      end;


    procedure tscannerfile.skipuntildirective;
      var
        found : longint;
        next_char_loaded : boolean;
      begin
         found:=0;
         next_char_loaded:=false;
         repeat
           case c of
             #10,
             #13 :
               linebreak;
             #26 :
               begin
                 reload;
                 if (c=#26) and not assigned(inputfile.next) then
                   end_of_file;
                 continue;
               end;
             '{' :
               begin
                 if (current_commentstyle in [comment_tp,comment_none]) then
                   begin
                     current_commentstyle:=comment_tp;
                     if (comment_level=0) then
                       found:=1;
                     inc_comment_level;
                   end;
               end;
             '*' :
               begin
                 if (current_commentstyle=comment_oldtp) then
                   begin
                     readchar;
                     if c=')' then
                       begin
                         dec_comment_level;
                         found:=0;
                         current_commentstyle:=comment_none;
                       end
                     else
                       next_char_loaded:=true;
                   end
                 else
                   found := 0;
               end;
             '}' :
               begin
                 if (current_commentstyle=comment_tp) then
                   begin
                     dec_comment_level;
                     if (comment_level=0) then
                       current_commentstyle:=comment_none;
                     found:=0;
                   end;
               end;
             '$' :
               begin
                 if found=1 then
                  found:=2;
               end;
             '''' :
               if (current_commentstyle=comment_none) then
                begin
                  repeat
                    readchar;
                    case c of
                      #26 :
                        end_of_file;
                      #10,#13 :
                        break;
                      '''' :
                        begin
                          readchar;
                          if c<>'''' then
                           begin
                             next_char_loaded:=true;
                             break;
                           end;
                        end;
                    end;
                  until false;
                end;
             '(' :
               begin
                 if (current_commentstyle=comment_none) then
                  begin
                    readchar;
                    if c='*' then
                     begin
                       readchar;
                       if c='$' then
                        begin
                          found:=2;
                          inc_comment_level;
                          current_commentstyle:=comment_oldtp;
                        end
                       else
                        begin
                          skipoldtpcomment(false);
                          next_char_loaded:=true;
                        end;
                     end
                    else
                     next_char_loaded:=true;
                  end
                 else
                  found:=0;
               end;
             '/' :
               begin
                 if (current_commentstyle=comment_none) then
                  begin
                    readchar;
                    if c='/' then
                     skipdelphicomment;
                    next_char_loaded:=true;
                  end
                 else
                  found:=0;
               end;
             else
               found:=0;
           end;
           if next_char_loaded then
             next_char_loaded:=false
           else
             readchar;
         until (found=2);
      end;


{****************************************************************************
                             Comment Handling
****************************************************************************}

    procedure tscannerfile.skipcomment(read_first_char:boolean);
      begin
        current_commentstyle:=comment_tp;
        if read_first_char then
          readchar;
        inc_comment_level;
      { handle compiler switches }
        if (c='$') then
         handledirectives;
      { handle_switches can dec comment_level,  }
        while (comment_level>0) do
         begin
           case c of
            '{' :
              inc_comment_level;
            '}' :
              dec_comment_level;
            '*' :
              { in iso mode, comments opened by a curly bracket can be closed by asterisk, round bracket }
              if m_iso in current_settings.modeswitches then
                begin
                  readchar;
                  if c=')' then
                    dec_comment_level
                  else
                    continue;
                end;
            #10,#13 :
              linebreak;
            #26 :
              begin
                reload;
                if (c=#26) and not assigned(inputfile.next) then
                  end_of_file;
                continue;
              end;
           end;
           readchar;
         end;
        current_commentstyle:=comment_none;
      end;


    procedure tscannerfile.skipdelphicomment;
      begin
        current_commentstyle:=comment_delphi;
        inc_comment_level;
        readchar;
        { this is not supported }
        if c='$' then
          Message(scan_w_wrong_styled_switch);
        { skip comment }
        while not (c in [#10,#13,#26]) do
          readchar;
        dec_comment_level;
        current_commentstyle:=comment_none;
      end;


    procedure tscannerfile.skipoldtpcomment(read_first_char:boolean);
      var
        found : longint;
      begin
        current_commentstyle:=comment_oldtp;
        inc_comment_level;
        { only load a char if last already processed,
          was cause of bug1634 PM }
        if read_first_char then
          readchar;
      { this is now supported }
        if (c='$') then
         handledirectives;
      { skip comment }
        while (comment_level>0) do
         begin
           found:=0;
           repeat
             case c of
               #26 :
                 begin
                   reload;
                   if (c=#26) and not assigned(inputfile.next) then
                     end_of_file;
                   continue;
                 end;
               #10,#13 :
                 begin
                   if found=4 then
                    inc_comment_level;
                   linebreak;
                   found:=0;
                 end;
               '*' :
                 begin
                   if found=3 then
                    found:=4
                   else
                    begin
                      if found=4 then
                        inc_comment_level;
                      found:=1;
                    end;
                 end;
               ')' :
                 begin
                   if found in [1,4] then
                    begin
                      dec_comment_level;
                      if comment_level=0 then
                       found:=2
                      else
                       found:=0;
                    end
                   else
                    found:=0;
                 end;
              '}' :
                { in iso mode, comments opened by asterisk, round bracket can be closed by a curly bracket }
                if m_iso in current_settings.modeswitches then
                  begin
                    dec_comment_level;
                    if comment_level=0 then
                      found:=2
                    else
                      found:=0;
                  end;
               '(' :
                 begin
                   if found=4 then
                    inc_comment_level;
                   found:=3;
                 end;
               else
                 begin
                   if found=4 then
                    inc_comment_level;
                   found:=0;
                 end;
             end;
             readchar;
           until (found=2);
         end;
        current_commentstyle:=comment_none;
      end;



{****************************************************************************
                               Token Scanner
****************************************************************************}

    procedure tscannerfile.readtoken(allowrecordtoken:boolean);
      var
        code    : integer;
        d : cardinal;
        len,
        low,high,mid : longint;
        w : word;
        m       : longint;
        mac     : tmacro;
        asciinr : string[33];
        iswidestring : boolean;
      label
         exit_label;
      begin
        flushpendingswitchesstate;

        { record tokens? }
        if allowrecordtoken and
           assigned(recordtokenbuf) then
          recordtoken;

        { replay tokens? }
        if assigned(replaytokenbuf) then
          begin
            replaytoken;
            goto exit_label;
          end;

      { was there already a token read, then return that token }
        if nexttoken<>NOTOKEN then
         begin
           setnexttoken;
           goto exit_label;
         end;

      { Skip all spaces and comments }
        repeat
          case c of
            '{' :
              skipcomment(true);
            #26 :
              begin
                reload;
                if (c=#26) and not assigned(inputfile.next) then
                  break;
              end;
            ' ',#9..#13 :
              begin
{$ifdef PREPROCWRITE}
                if parapreprocess then
                 begin
                   if c=#10 then
                    preprocfile.eolfound:=true
                   else
                    preprocfile.spacefound:=true;
                 end;
{$endif PREPROCWRITE}
                skipspace;
              end
            else
              break;
          end;
        until false;

      { Save current token position, for EOF its already loaded }
        if c<>#26 then
          gettokenpos;

      { Check first for a identifier/keyword, this is 20+% faster (PFV) }
        if c in ['A'..'Z','a'..'z','_'] then
         begin
           readstring;
           token:=_ID;
           idtoken:=_ID;
         { keyword or any other known token,
           pattern is always uppercased }
           if (pattern[1]<>'_') and (length(pattern) in [tokenlenmin..tokenlenmax]) then
            begin
              low:=ord(tokenidx^[length(pattern),pattern[1]].first);
              high:=ord(tokenidx^[length(pattern),pattern[1]].last);
              while low<high do
               begin
                 mid:=(high+low+1) shr 1;
                 if pattern<tokeninfo^[ttoken(mid)].str then
                  high:=mid-1
                 else
                  low:=mid;
               end;
              with tokeninfo^[ttoken(high)] do
                if pattern=str then
                  begin
                    if (keyword*current_settings.modeswitches)<>[] then
                      if op=NOTOKEN then
                        token:=ttoken(high)
                      else
                        token:=op;
                    idtoken:=ttoken(high);
                  end;
            end;
         { Only process identifiers and not keywords }
           if token=_ID then
            begin
            { this takes some time ... }
              if (cs_support_macro in current_settings.moduleswitches) then
               begin
                 mac:=tmacro(search_macro(pattern));
                 if assigned(mac) and (not mac.is_compiler_var) and (assigned(mac.buftext)) then
                  begin
                    if yylexcount<max_macro_nesting then
                     begin
                       mac.is_used:=true;
                       inc(yylexcount);
                       substitutemacro(pattern,mac.buftext,mac.buflen,
                         mac.fileinfo.line,mac.fileinfo.fileindex);
                     { handle empty macros }
                       if c=#0 then
                         reload;
                       readtoken(false);
                       { that's all folks }
                       dec(yylexcount);
                       exit;
                     end
                    else
                     Message(scan_w_macro_too_deep);
                  end;
               end;
            end;
         { return token }
           goto exit_label;
         end
        else
         begin
           idtoken:=_NOID;
           case c of

             '$' :
               begin
                 readnumber;
                 token:=_INTCONST;
                 goto exit_label;
               end;

             '%' :
               begin
                 if not(m_fpc in current_settings.modeswitches) then
                  Illegal_Char(c)
                 else
                  begin
                    readnumber;
                    token:=_INTCONST;
                    goto exit_label;
                  end;
               end;

             '&' :
               begin
                 if [m_fpc,m_delphi] * current_settings.modeswitches <> [] then
                  begin
                    readnumber;
                    if length(pattern)=1 then
                      begin
                        { does really an identifier follow? }
                        if not (c in ['_','A'..'Z','a'..'z']) then
                          message2(scan_f_syn_expected,tokeninfo^[_ID].str,c);
                        readstring;
                        token:=_ID;
                        idtoken:=_ID;
                      end
                    else
                      token:=_INTCONST;
                    goto exit_label;
                  end
                 else if m_mac in current_settings.modeswitches then
                  begin
                    readchar;
                    token:=_AMPERSAND;
                    goto exit_label;
                  end
                 else
                  Illegal_Char(c);
               end;

             '0'..'9' :
               begin
                 readnumber;
                 if (c in ['.','e','E']) then
                  begin
                  { first check for a . }
                    if c='.' then
                     begin
                       cachenexttokenpos;
                       readchar;
                       { is it a .. from a range? }
                       case c of
                         '.' :
                           begin
                             readchar;
                             token:=_INTCONST;
                             nexttoken:=_POINTPOINT;
                             goto exit_label;
                           end;
                         ')' :
                           begin
                             readchar;
                             token:=_INTCONST;
                             nexttoken:=_RECKKLAMMER;
                             goto exit_label;
                           end;
                         '0'..'9' :
                           begin
                             { insert the number after the . }
                             pattern:=pattern+'.';
                             while c in ['0'..'9'] do
                              begin
                                pattern:=pattern+c;
                                readchar;
                              end;
                           end;
                         else
                           begin
                             token:=_INTCONST;
                             nexttoken:=_POINT;
                             goto exit_label;
                           end;
                       end;
                      end;
                  { E can also follow after a point is scanned }
                    if c in ['e','E'] then
                     begin
                       pattern:=pattern+'E';
                       readchar;
                       if c in ['-','+'] then
                        begin
                          pattern:=pattern+c;
                          readchar;
                        end;
                       if not(c in ['0'..'9']) then
                        Illegal_Char(c);
                       while c in ['0'..'9'] do
                        begin
                          pattern:=pattern+c;
                          readchar;
                        end;
                     end;
                    token:=_REALNUMBER;
                    goto exit_label;
                  end;
                 token:=_INTCONST;
                 goto exit_label;
               end;

             ';' :
               begin
                 readchar;
                 token:=_SEMICOLON;
                 goto exit_label;
               end;

             '[' :
               begin
                 readchar;
                 token:=_LECKKLAMMER;
                 goto exit_label;
               end;

             ']' :
               begin
                 readchar;
                 token:=_RECKKLAMMER;
                 goto exit_label;
               end;

             '(' :
               begin
                 readchar;
                 case c of
                   '*' :
                     begin
                       skipoldtpcomment(true);
                       readtoken(false);
                       exit;
                     end;
                   '.' :
                     begin
                       readchar;
                       token:=_LECKKLAMMER;
                       goto exit_label;
                     end;
                 end;
                 token:=_LKLAMMER;
                 goto exit_label;
               end;

             ')' :
               begin
                 readchar;
                 token:=_RKLAMMER;
                 goto exit_label;
               end;

             '+' :
               begin
                 readchar;
                 if (c='=') and (cs_support_c_operators in current_settings.moduleswitches) then
                  begin
                    readchar;
                    token:=_PLUSASN;
                    goto exit_label;
                  end;
                 token:=_PLUS;
                 goto exit_label;
               end;

             '-' :
               begin
                 readchar;
                 if (c='=') and (cs_support_c_operators in current_settings.moduleswitches) then
                  begin
                    readchar;
                    token:=_MINUSASN;
                    goto exit_label;
                  end;
                 token:=_MINUS;
                 goto exit_label;
               end;

             ':' :
               begin
                 readchar;
                 if c='=' then
                  begin
                    readchar;
                    token:=_ASSIGNMENT;
                    goto exit_label;
                  end;
                 token:=_COLON;
                 goto exit_label;
               end;

             '*' :
               begin
                 readchar;
                 if (c='=') and (cs_support_c_operators in current_settings.moduleswitches) then
                  begin
                    readchar;
                    token:=_STARASN;
                  end
                 else
                  if c='*' then
                   begin
                     readchar;
                     token:=_STARSTAR;
                   end
                 else
                  token:=_STAR;
                 goto exit_label;
               end;

             '/' :
               begin
                 readchar;
                 case c of
                   '=' :
                     begin
                       if (cs_support_c_operators in current_settings.moduleswitches) then
                        begin
                          readchar;
                          token:=_SLASHASN;
                          goto exit_label;
                        end;
                     end;
                   '/' :
                     begin
                       skipdelphicomment;
                       readtoken(false);
                       exit;
                     end;
                 end;
                 token:=_SLASH;
                 goto exit_label;
               end;

             '|' :
               if m_mac in current_settings.modeswitches then
                begin
                  readchar;
                  token:=_PIPE;
                  goto exit_label;
                end
               else
                Illegal_Char(c);

             '=' :
               begin
                 readchar;
                 token:=_EQ;
                 goto exit_label;
               end;

             '.' :
               begin
                 readchar;
                 case c of
                   '.' :
                     begin
                       readchar;
                       case c of
                         '.' :
                         begin
                           readchar;
                           token:=_POINTPOINTPOINT;
                           goto exit_label;
                         end;
                       else
                         begin
                           token:=_POINTPOINT;
                           goto exit_label;
                         end;
                       end;
                     end;
                   ')' :
                     begin
                       readchar;
                       token:=_RECKKLAMMER;
                       goto exit_label;
                     end;
                 end;
                 token:=_POINT;
                 goto exit_label;
               end;

             '@' :
               begin
                 readchar;
                 token:=_KLAMMERAFFE;
                 goto exit_label;
               end;

             ',' :
               begin
                 readchar;
                 token:=_COMMA;
                 goto exit_label;
               end;

             '''','#','^' :
               begin
                 len:=0;
                 cstringpattern:='';
                 iswidestring:=false;
                 if c='^' then
                  begin
                    readchar;
                    c:=upcase(c);
                    if (block_type in [bt_type,bt_const_type,bt_var_type]) or
                       (lasttoken=_ID) or (lasttoken=_NIL) or (lasttoken=_OPERATOR) or
                       (lasttoken=_RKLAMMER) or (lasttoken=_RECKKLAMMER) or (lasttoken=_CARET) then
                     begin
                       token:=_CARET;
                       goto exit_label;
                     end
                    else
                     begin
                       inc(len);
                       setlength(cstringpattern,256);
                       if c<#64 then
                         cstringpattern[len]:=chr(ord(c)+64)
                       else
                         cstringpattern[len]:=chr(ord(c)-64);
                       readchar;
                     end;
                  end;
                 repeat
                   case c of
                     '#' :
                       begin
                         readchar; { read # }
                         case c of
                           '$':
                             begin
                               readchar; { read leading $ }
                               asciinr:='$';
                               while (upcase(c) in ['A'..'F','0'..'9']) and (length(asciinr)<=7) do
                                 begin
                                   asciinr:=asciinr+c;
                                   readchar;
                                 end;
                             end;
                           '&':
                             begin
                               readchar; { read leading $ }
                               asciinr:='&';
                               while (upcase(c) in ['0'..'7']) and (length(asciinr)<=8) do
                                 begin
                                   asciinr:=asciinr+c;
                                   readchar;
                                 end;
                             end;
                           '%':
                             begin
                               readchar; { read leading $ }
                               asciinr:='%';
                               while (upcase(c) in ['0','1']) and (length(asciinr)<=22) do
                                 begin
                                   asciinr:=asciinr+c;
                                   readchar;
                                 end;
                             end;
                           else
                             begin
                               asciinr:='';
                               while (c in ['0'..'9']) and (length(asciinr)<=8) do
                                 begin
                                   asciinr:=asciinr+c;
                                   readchar;
                                 end;
                             end;
                         end;
                         val(asciinr,m,code);
                         if (asciinr='') or (code<>0) then
                           Message(scan_e_illegal_char_const)
                         else if (m<0) or (m>255) or (length(asciinr)>3) then
                           begin
                              if (m>=0) and (m<=$10FFFF) then
                                begin
                                  if not iswidestring then
                                   begin
                                     if len>0 then
                                       ascii2unicode(@cstringpattern[1],len,current_settings.sourcecodepage,patternw)
                                     else
                                       ascii2unicode(nil,len,current_settings.sourcecodepage,patternw);
                                     iswidestring:=true;
                                     len:=0;
                                   end;
                                  if m<=$FFFF then
                                    concatwidestringchar(patternw,tcompilerwidechar(m))
                                  else
                                    begin
                                      { split into surrogate pair }
                                      dec(m,$10000);
                                      concatwidestringchar(patternw,tcompilerwidechar((m shr 10) + $D800));
                                      concatwidestringchar(patternw,tcompilerwidechar((m and $3FF) + $DC00));
                                    end;
                                end
                              else
                                Message(scan_e_illegal_char_const)
                           end
                         else if iswidestring then
                           concatwidestringchar(patternw,asciichar2unicode(char(m)))
                         else
                           begin
                             if len>=length(cstringpattern) then
                               setlength(cstringpattern,length(cstringpattern)+256);
                              inc(len);
                              cstringpattern[len]:=chr(m);
                           end;
                       end;
                     '''' :
                       begin
                         repeat
                           readchar;
                           case c of
                             #26 :
                               end_of_file;
                             #10,#13 :
                               Message(scan_f_string_exceeds_line);
                             '''' :
                               begin
                                 readchar;
                                 if c<>'''' then
                                  break;
                               end;
                           end;
                           { interpret as utf-8 string? }
                           if (ord(c)>=$80) and (current_settings.sourcecodepage=CP_UTF8) then
                             begin
                               { convert existing string to an utf-8 string }
                               if not iswidestring then
                                 begin
                                   if len>0 then
                                     ascii2unicode(@cstringpattern[1],len,current_settings.sourcecodepage,patternw)
                                   else
                                     ascii2unicode(nil,len,current_settings.sourcecodepage,patternw);
                                   iswidestring:=true;
                                   len:=0;
                                 end;
                               { four chars }
                               if (ord(c) and $f0)=$f0 then
                                 begin
                                   { this always represents a surrogate pair, so
                                     read as 32-bit value and then split into
                                     the corresponding pair of two wchars }
                                   d:=ord(c) and $f;
                                   readchar;
                                   if (ord(c) and $c0)<>$80 then
                                     message(scan_e_utf8_malformed);
                                   d:=(d shl 6) or (ord(c) and $3f);
                                   readchar;
                                   if (ord(c) and $c0)<>$80 then
                                     message(scan_e_utf8_malformed);
                                   d:=(d shl 6) or (ord(c) and $3f);
                                   readchar;
                                   if (ord(c) and $c0)<>$80 then
                                     message(scan_e_utf8_malformed);
                                   d:=(d shl 6) or (ord(c) and $3f);
                                   if d<$10000 then
                                     message(scan_e_utf8_malformed);
                                   d:=d-$10000;
                                   { high surrogate }
                                   w:=$d800+(d shr 10);
                                   concatwidestringchar(patternw,w);
                                   { low surrogate }
                                   w:=$dc00+(d and $3ff);
                                   concatwidestringchar(patternw,w);
                                 end
                               { three chars }
                               else if (ord(c) and $e0)=$e0 then
                                 begin
                                   w:=ord(c) and $f;
                                   readchar;
                                   if (ord(c) and $c0)<>$80 then
                                     message(scan_e_utf8_malformed);
                                   w:=(w shl 6) or (ord(c) and $3f);
                                   readchar;
                                   if (ord(c) and $c0)<>$80 then
                                     message(scan_e_utf8_malformed);
                                   w:=(w shl 6) or (ord(c) and $3f);
                                   concatwidestringchar(patternw,w);
                                 end
                               { two chars }
                               else if (ord(c) and $c0)<>0 then
                                 begin
                                   w:=ord(c) and $1f;
                                   readchar;
                                   if (ord(c) and $c0)<>$80 then
                                     message(scan_e_utf8_malformed);
                                   w:=(w shl 6) or (ord(c) and $3f);
                                   concatwidestringchar(patternw,w);
                                 end
                               { illegal }
                               else if (ord(c) and $80)<>0 then
                                 message(scan_e_utf8_malformed)
                               else
                                 concatwidestringchar(patternw,tcompilerwidechar(c))
                             end
                           else if iswidestring then
                             begin
                               if current_settings.sourcecodepage=CP_UTF8 then
                                 concatwidestringchar(patternw,ord(c))
                               else
                                 concatwidestringchar(patternw,asciichar2unicode(c))
                             end
                           else
                             begin
                               if len>=length(cstringpattern) then
                                 setlength(cstringpattern,length(cstringpattern)+256);
                                inc(len);
                                cstringpattern[len]:=c;
                             end;
                         until false;
                       end;
                     '^' :
                       begin
                         readchar;
                         c:=upcase(c);
                         if c<#64 then
                          c:=chr(ord(c)+64)
                         else
                          c:=chr(ord(c)-64);

                         if iswidestring then
                           concatwidestringchar(patternw,asciichar2unicode(c))
                         else
                           begin
                             if len>=length(cstringpattern) then
                               setlength(cstringpattern,length(cstringpattern)+256);
                              inc(len);
                              cstringpattern[len]:=c;
                           end;

                         readchar;
                       end;
                     else
                      break;
                   end;
                 until false;
                 { strings with length 1 become const chars }
                 if iswidestring then
                   begin
                     if patternw^.len=1 then
                       token:=_CWCHAR
                     else
                       token:=_CWSTRING;
                   end
                 else
                   begin
                     setlength(cstringpattern,len);
                     if length(cstringpattern)=1 then
                       begin
                         token:=_CCHAR;
                         pattern:=cstringpattern;
                       end
                     else
                       token:=_CSTRING;
                   end;
                 goto exit_label;
               end;

             '>' :
               begin
                 readchar;
                 if (block_type in [bt_type,bt_var_type,bt_const_type]) then
                   token:=_RSHARPBRACKET
                 else
                   begin
                     case c of
                       '=' :
                         begin
                           readchar;
                           token:=_GTE;
                           goto exit_label;
                         end;
                       '>' :
                         begin
                           readchar;
                           token:=_OP_SHR;
                           goto exit_label;
                         end;
                       '<' :
                         begin { >< is for a symetric diff for sets }
                           readchar;
                           token:=_SYMDIF;
                           goto exit_label;
                         end;
                     end;
                     token:=_GT;
                   end;
                 goto exit_label;
               end;

             '<' :
               begin
                 readchar;
                 if (block_type in [bt_type,bt_var_type,bt_const_type]) then
                   token:=_LSHARPBRACKET
                 else
                   begin
                     case c of
                       '>' :
                         begin
                           readchar;
                           token:=_NE;
                           goto exit_label;
                         end;
                       '=' :
                         begin
                           readchar;
                           token:=_LTE;
                           goto exit_label;
                         end;
                       '<' :
                         begin
                           readchar;
                           token:=_OP_SHL;
                           goto exit_label;
                         end;
                     end;
                     token:=_LT;
                   end;
                 goto exit_label;
               end;

             #26 :
               begin
                 token:=_EOF;
                 checkpreprocstack;
                 goto exit_label;
               end;
             else
               Illegal_Char(c);
           end;
        end;
exit_label:
        lasttoken:=token;
      end;


    function tscannerfile.readpreproc:ttoken;
      var
        low,high,mid: longint;
        optoken: ttoken;
      begin
         skipspace;
         case c of
           '_',
           'A'..'Z',
           'a'..'z' :
             begin
               readstring;
               optoken:=_ID;
               if (pattern[1]<>'_') and (length(pattern) in [tokenlenmin..tokenlenmax]) then
                begin
                  low:=ord(tokenidx^[length(pattern),pattern[1]].first);
                  high:=ord(tokenidx^[length(pattern),pattern[1]].last);
                  while low<high do
                   begin
                     mid:=(high+low+1) shr 1;
                     if pattern<tokeninfo^[ttoken(mid)].str then
                      high:=mid-1
                     else
                      low:=mid;
                   end;
                  with tokeninfo^[ttoken(high)] do
                    if pattern=str then
                      begin
                        if (keyword*current_settings.modeswitches)<>[] then
                          if op=NOTOKEN then
                            optoken:=ttoken(high)
                          else
                            optoken:=op;
                      end;
                  if not (optoken in preproc_operators) then
                    optoken:=_ID;
                end;
               current_scanner.preproc_pattern:=pattern;
               readpreproc:=optoken;
             end;
           '''' :
             begin
               readquotedstring;
               current_scanner.preproc_pattern:=cstringpattern;
               readpreproc:=_CSTRING;
             end;
           '0'..'9' :
             begin
               readnumber;
               if (c in ['.','e','E']) then
                 begin
                   { first check for a . }
                   if c='.' then
                     begin
                       readchar;
                       if c in ['0'..'9'] then
                         begin
                           { insert the number after the . }
                           pattern:=pattern+'.';
                           while c in ['0'..'9'] do
                             begin
                               pattern:=pattern+c;
                               readchar;
                             end;
                         end
                       else
                         Illegal_Char(c);
                     end;
                  { E can also follow after a point is scanned }
                   if c in ['e','E'] then
                     begin
                       pattern:=pattern+'E';
                       readchar;
                       if c in ['-','+'] then
                         begin
                           pattern:=pattern+c;
                           readchar;
                         end;
                       if not(c in ['0'..'9']) then
                         Illegal_Char(c);
                       while c in ['0'..'9'] do
                         begin
                           pattern:=pattern+c;
                           readchar;
                         end;
                     end;
                   readpreproc:=_REALNUMBER;
                 end
               else
                 readpreproc:=_INTCONST;
               current_scanner.preproc_pattern:=pattern;
             end;
           '$','%':
             begin
               readnumber;
               current_scanner.preproc_pattern:=pattern;
               readpreproc:=_INTCONST;
             end;
           '&' :
             begin
                readnumber;
                if length(pattern)=1 then
                  begin
                    readstring;
                    readpreproc:=_ID;
                  end
                else
                  readpreproc:=_INTCONST;
               current_scanner.preproc_pattern:=pattern;
             end;
           '.' :
             begin
               readchar;
               readpreproc:=_POINT;
             end;
           ',' :
             begin
               readchar;
               readpreproc:=_COMMA;
             end;
           '}' :
             begin
               readpreproc:=_END;
             end;
           '(' :
             begin
               readchar;
               readpreproc:=_LKLAMMER;
             end;
           ')' :
             begin
               readchar;
               readpreproc:=_RKLAMMER;
             end;
           '[' :
             begin
               readchar;
               readpreproc:=_LECKKLAMMER;
             end;
           ']' :
             begin
               readchar;
               readpreproc:=_RECKKLAMMER;
             end;
           '+' :
             begin
               readchar;
               readpreproc:=_PLUS;
             end;
           '-' :
             begin
               readchar;
               readpreproc:=_MINUS;
             end;
           '*' :
             begin
               readchar;
               readpreproc:=_STAR;
             end;
           '/' :
             begin
               readchar;
               readpreproc:=_SLASH;
             end;
           '=' :
             begin
               readchar;
               readpreproc:=_EQ;
             end;
           '>' :
             begin
               readchar;
               if c='=' then
                 begin
                   readchar;
                   readpreproc:=_GTE;
                 end
               else
                 readpreproc:=_GT;
             end;
           '<' :
             begin
               readchar;
               case c of
                 '>' :
                   begin
                     readchar;
                     readpreproc:=_NE;
                   end;
                 '=' :
                   begin
                     readchar;
                     readpreproc:=_LTE;
                   end;
                 else
                   readpreproc:=_LT;
               end;
             end;
           #26 :
             begin
               readpreproc:=_EOF;
               checkpreprocstack;
             end;
           else
             begin
               Illegal_Char(c);
               readpreproc:=NOTOKEN;
             end;
         end;
      end;


    function tscannerfile.readpreprocint(var value:int64;const place:string):boolean;
      var
        hs : texprvalue;
      begin
        hs:=preproc_comp_expr;
        if hs.isInt then
          begin
            value:=hs.asInt64;
            result:=true;
          end
        else
          begin
            hs.error('Integer',place);
            result:=false;
          end;
        hs.free;
      end;


    function tscannerfile.asmgetchar : char;
      begin
         readchar;
         repeat
           case c of
             #26 :
               begin
                 reload;
                 if (c=#26) and not assigned(inputfile.next) then
                   end_of_file;
                 continue;
               end;
             else
               begin
                 asmgetchar:=c;
                 exit;
               end;
           end;
         until false;
      end;


{*****************************************************************************
                                   Helpers
*****************************************************************************}

    procedure AddDirective(const s:string; dm: tdirectivemode; p:tdirectiveproc);
      begin
        if dm in [directive_all, directive_turbo] then
          tdirectiveitem.create(turbo_scannerdirectives,s,p);
        if dm in [directive_all, directive_mac] then
          tdirectiveitem.create(mac_scannerdirectives,s,p);
      end;

    procedure AddConditional(const s:string; dm: tdirectivemode; p:tdirectiveproc);
      begin
        if dm in [directive_all, directive_turbo] then
          tdirectiveitem.createcond(turbo_scannerdirectives,s,p);
        if dm in [directive_all, directive_mac] then
          tdirectiveitem.createcond(mac_scannerdirectives,s,p);
      end;

{*****************************************************************************
                                Initialization
*****************************************************************************}

    procedure InitScanner;
      begin
        InitWideString(patternw);
        turbo_scannerdirectives:=TFPHashObjectList.Create;
        mac_scannerdirectives:=TFPHashObjectList.Create;

        { Common directives and conditionals }
        AddDirective('I',directive_all, @dir_include);
        AddDirective('DEFINE',directive_all, @dir_define);
        AddDirective('UNDEF',directive_all, @dir_undef);

        AddConditional('IF',directive_all, @dir_if);
        AddConditional('IFDEF',directive_all, @dir_ifdef);
        AddConditional('IFNDEF',directive_all, @dir_ifndef);
        AddConditional('ELSE',directive_all, @dir_else);
        AddConditional('ELSEIF',directive_all, @dir_elseif);
        AddConditional('ENDIF',directive_all, @dir_endif);

        { Directives and conditionals for all modes except mode macpas}
        AddDirective('INCLUDE',directive_turbo, @dir_include);
        AddDirective('LIBPREFIX',directive_turbo, @dir_libprefix);
        AddDirective('LIBSUFFIX',directive_turbo, @dir_libsuffix);
        AddDirective('EXTENSION',directive_turbo, @dir_extension);

        AddConditional('IFEND',directive_turbo, @dir_endif);
        AddConditional('IFOPT',directive_turbo, @dir_ifopt);

        { Directives and conditionals for mode macpas: }
        AddDirective('SETC',directive_mac, @dir_setc);
        AddDirective('DEFINEC',directive_mac, @dir_definec);
        AddDirective('UNDEFC',directive_mac, @dir_undef);

        AddConditional('IFC',directive_mac, @dir_if);
        AddConditional('ELSEC',directive_mac, @dir_else);
        AddConditional('ELIFC',directive_mac, @dir_elseif);
        AddConditional('ENDC',directive_mac, @dir_endif);
      end;


    procedure DoneScanner;
      begin
        turbo_scannerdirectives.Free;
        mac_scannerdirectives.Free;
        DoneWideString(patternw);
      end;

end.
