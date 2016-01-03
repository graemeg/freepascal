{$mode objfpc}
{$h+}

unit testu;

Interface

{ ---------------------------------------------------------------------
    utility functions, shared by several programs of the test suite
  ---------------------------------------------------------------------}

type
  TVerboseLevel=(V_Abort,V_Error,V_Warning,V_Normal,V_Debug,V_SQL);

  TConfig = record
    NeedOptions,
    DelOptions,
    NeedCPU,
    SkipCPU,
    SkipEmu,
    NeedTarget,
    SkipTarget,
    MinVersion,
    MaxVersion,
    KnownRunNote,
    KnownCompileNote,
    RecompileOpt: string;
    ResultCode    : longint;
    KnownRunError : longint;
    KnownCompileError : longint;
    NeedRecompile : boolean;
    NeedLibrary   : boolean;
    NeededAfter   : boolean;
    IsInteractive : boolean;
    IsKnownRunError,
    IsKnownCompileError : boolean;
    NoRun         : boolean;
    UsesGraph     : boolean;
    ShouldFail    : boolean;
    Timeout       : longint;
    Category      : string;
    Note          : string;
    Files         : string;
    WpoParas      : string;
    WpoPasses     : longint;
    DelFiles      : string;
  end;

Const
  DoVerbose : boolean = false;
  DoSQL     : boolean = false;

procedure TrimB(var s:string);
procedure TrimE(var s:string);
function upper(const s : string) : string;
procedure Verbose(lvl:TVerboseLevel;const s:string);
function GetConfig(const fn:string;var r:TConfig):boolean;
Function GetFileContents (FN : String) : String;

Implementation

procedure Verbose(lvl:TVerboseLevel;const s:string);
begin
  case lvl of
    V_Normal :
      writeln(s);
    V_Debug :
      if DoVerbose then
       writeln('Debug: ',s);
    V_SQL :
      if DoSQL then
       writeln('SQL: ',s);
    V_Warning :
      writeln('Warning: ',s);
    V_Error :
      begin
        writeln('Error: ',s);
        halt(1);
      end;
    V_Abort :
      begin
        writeln('Abort: ',s);
        halt(0);
      end;
  end;
end;

procedure TrimB(var s:string);
begin
  while (s<>'') and (s[1] in [' ',#9]) do
   delete(s,1,1);
end;


procedure TrimE(var s:string);
begin
  while (s<>'') and (s[length(s)] in [' ',#9]) do
   delete(s,length(s),1);
end;


function upper(const s : string) : string;
var
  i,l  : longint;

begin
  L:=Length(S);
  SetLength(upper,l);
  for i:=1 to l do
    if s[i] in ['a'..'z'] then
     upper[i]:=char(byte(s[i])-32)
    else
     upper[i]:=s[i];
end;

function GetConfig(const fn:string;var r:TConfig):boolean;
var
  t : text;
  part,code : integer;
  l : longint;
  s,res : string;

  function GetEntry(const entry:string):boolean;
  var
    i : longint;
  begin
    Getentry:=false;
    Res:='';
    if Upper(Copy(s,1,length(entry)))=Upper(entry) then
     begin
       Delete(s,1,length(entry));
       TrimB(s);
       if (s<>'') then
        begin
          if (s[1]='=') then
           begin
             delete(s,1,1);
             i:=pos('}',s);
             if i=0 then
              i:=255
             else
              dec(i);
             res:=Copy(s,1,i);
             TrimB(res);
             TrimE(res);
           end;
          Verbose(V_Debug,'Config: '+Entry+' = "'+Res+'"');
          GetEntry:=true;
        end;
     end;
  end;

begin
  FillChar(r,sizeof(r),0);
  GetConfig:=false;
  Verbose(V_Debug,'Reading '+fn);
  assign(t,fn);
  {$I-}
   reset(t);
  {$I+}
  if ioresult<>0 then
   begin
     Verbose(V_Error,'Can''t open '+fn);
     exit;
   end;
  r.Note:='';
  while not eof(t) do
   begin
     readln(t,s);
     if Copy(s,1,3)=#$EF#$BB#$BF then
       delete(s,1,3);
<<<<<<< HEAD
<<<<<<< HEAD
     TrimB(s);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     if s<>'' then
      begin
        if s[1]='{' then
         begin
           delete(s,1,1);
           TrimB(s);
           if (s<>'') and (s[1]='%') then
            begin
              delete(s,1,1);
              if GetEntry('OPT') then
               r.NeedOptions:=res
              else
               if GetEntry('DELOPT') then
                r.DelOptions:=res
              else
               if GetEntry('TARGET') then
                r.NeedTarget:=res
              else
               if GetEntry('SKIPTARGET') then
                r.SkipTarget:=res
              else
               if GetEntry('CPU') then
                r.NeedCPU:=res
              else
               if GetEntry('SKIPCPU') then
                r.SkipCPU:=res
              else
               if GetEntry('SKIPEMU') then
                r.SkipEmu:=res
              else
               if GetEntry('VERSION') then
                r.MinVersion:=res
              else
               if GetEntry('MAXVERSION') then
                r.MaxVersion:=res
              else
               if GetEntry('RESULT') then
                Val(res,r.ResultCode,code)
              else
               if GetEntry('GRAPH') then
                r.UsesGraph:=true
              else
               if GetEntry('FAIL') then
                r.ShouldFail:=true
              else
               if GetEntry('RECOMPILE') then
	        begin
                  r.NeedRecompile:=true;
		  r.RecompileOpt:=res;
		end
              else
               if GetEntry('NORUN') then
                r.NoRun:=true
              else
               if GetEntry('NEEDLIBRARY') then
                r.NeedLibrary:=true
              else
               if GetEntry('NEEDEDAFTER') then
                r.NeededAfter:=true
              else
               if GetEntry('KNOWNRUNERROR') then
                begin
                  r.IsKnownRunError:=true;
                  if res<>'' then
                    begin
                      val(res,l,code);
                      if code>1 then
                        begin
                          part:=code;
                          val(copy(res,1,code-1),l,code);
                          delete(res,1,part);
                        end;
                      if code=0 then
                        r.KnownRunError:=l;
                      if res<>'' then
                        r.KnownRunNote:=res;
                    end;
                end
              else
               if GetEntry('KNOWNCOMPILEERROR') then
                begin
                  r.IsKnownCompileError:=true;
                  if res<>'' then
                    begin
                      val(res,l,code);
                      if code>1 then
                        begin
                          part:=code;
                          val(copy(res,1,code-1),l,code);
                          delete(res,1,part);
                        end;
                      if code=0 then
                        r.KnownCompileError:=l;
                      if res<>'' then
                        r.KnownCompileNote:=res;
                    end;
                end
              else
               if GetEntry('INTERACTIVE') then
                r.IsInteractive:=true
              else
               if GetEntry('NOTE') then
                begin
                  R.Note:='Note: '+res;
                  Verbose(V_Normal,r.Note);
                end
              else
               if GetEntry('TIMEOUT') then
                Val(res,r.Timeout,code)
              else
<<<<<<< HEAD
<<<<<<< HEAD
               if GetEntry('FILES') then
                r.Files:=res
              else
                if GetEntry('WPOPARAS') then
                 r.wpoparas:=res
              else
                if GetEntry('WPOPASSES') then
                 val(res,r.wpopasses,code)
              else
                if GetEntry('DELFILES') then
                  r.DelFiles:=res
              else
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
               Verbose(V_Error,'Unknown entry: '+s);
            end;
         end
        else
         break;
      end;
   end;
  close(t);
  GetConfig:=true;
end;

Function GetFileContents (FN : String) : String;

Var
  F : Text;
  S : String;

begin
  Result:='';
  Assign(F,FN);
  {$I-}
  Reset(F);
  If IOResult<>0 then
    Exit;
  {$I+}
  While Not(EOF(F)) do
    begin
    ReadLn(F,S);
    Result:=Result+S+LineEnding;
    end;
  Close(F);
end;

end.
