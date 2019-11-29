{
    This program is part of the Free Pascal run time library.
    Copyright (c) 1998-2002 by Peter Vreman

    Convert a .msg file to an .inc file with a const array of char
    And for the lazy docwriters it can also generate some TeX output

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program msg2inc;

{$ifdef unix}
  {$define EOL_ONE_CHAR}
{$endif unix}
{$ifdef amiga}
  {$define EOL_ONE_CHAR}
{$endif amiga}
{$ifdef morphos}
  {$define EOL_ONE_CHAR}
{$endif}
{$ifdef macos}
  {$define EOL_ONE_CHAR}
{$endif}

const
  version='1.00';
{$ifdef EOL_ONE_CHAR}
  eollen=1;
{$else}
  eollen=2;
{$endif}
  msgparts = 20;

type
  TMode=(M_Char,M_Tex,M_Intel,M_String,M_Renumber);
var
  InFile,
  OutFile,
  OutName    : string;
  Mode       : TMode;
  TexHeader  : boolean;

  MsgTxt     : pchar;
  EnumTxt    : pchar;
  enumsize,
  msgsize    : longint;

  msgidxmax  : array[1..msgparts] of longint;
  msgs       : array[0..msgparts,0..999] of boolean;
  msgcodepage: TSystemCodePage;

procedure LoadMsgFile(const fn:string);
var
  f       : text;
  error,
  multiline : boolean;
  code : word;
  numpart,numidx,
  line,i,j,num  : longint;
  ptxt,
  penum   : pchar;
  number,
  s,s1    : string;

  procedure err(const msgstr:string);
  begin
    writeln('error in line ',line,': ',msgstr);
    error:=true;
  end;

begin
  Writeln('Loading messagefile ',fn);
  msgcodepage:=CP_ACP;
{Read the message file}
  assign(f,fn);
  {$push} {$I-}
   reset(f);
  {$pop}
  if ioresult<>0 then
   begin
     WriteLn('fatal error: '+fn+' not found');
     halt(1);
   end;
{ First parse the file and count bytes needed }
  fillchar(msgidxmax,sizeof(msgidxmax),0);
  fillchar(msgs,sizeof(msgs),0);
  error:=false;
  line:=0;
  multiline:=false;
  msgsize:=0;
  while not eof(f) do
   begin
     readln(f,s);
     inc(line);
     if multiline then
      begin
        if s=']' then
         multiline:=false
        else if (s='') or (s[1] <> '#') then
         inc(msgsize,length(s)+1); { +1 for linebreak }
      end
     else
      begin
        if (s<>'') and not(s[1] in ['#',';','%']) then
         begin
           i:=pos('=',s);
           if i>0 then
            begin
              j:=i+1;
              if not(s[j] in ['0'..'9']) then
               err('no number found')
              else
               begin
                 while (s[j] in ['0'..'9']) do
                  inc(j);
               end;
              if j-i-1<>5 then
               err('number length is not 5');
              number:=Copy(s,i+1,j-i-1);
              { update the max index }
              val(number,num,code);
              numpart:=num div 1000;
              if numpart=0 then
               err('number should be > 1000');
              if code<>0 then
               err('illegal number: '+s);
              numidx:=num mod 1000;
              { duplicate ? }
              if msgs[numpart,numidx] then
               err('duplicate number found');
              msgs[numpart,numidx]:=true;
              { check range }
              if numpart > msgparts then
               err('number is to large')
              else
               if numidx > msgidxmax[numpart] then
                msgidxmax[numpart]:=numidx;
              if s[j+1]='[' then
               begin
                 inc(msgsize,j-i);
                 multiline:=true
               end
              else
               inc(msgsize,length(s)-i+1);
              inc(enumsize,j);
            end
           else
            err('no = found');
         end
        else if (Length(s)>11) and (Copy(s,1,11)='# CodePage ') then
         begin
           val(Copy(s,12,Length(s)-11),msgcodepage,code);
           if code<>0 then
             err('illegal code page number: '+s);
         end;
      end;
   end;
  if multiline then
   err('still in multiline mode');
  if error then
   begin
     close(f);
     writeln('aborting');
     halt(1);
   end;
{ alloc memory }
  getmem(msgtxt,msgsize);
{ no linebreak after last entry }
  dec(msgsize);
  ptxt:=msgtxt;
  getmem(enumtxt,enumsize);
  penum:=enumtxt;
{ now read the buffer in mem }
  reset(f);
  while not eof(f) do
   begin
     readln(f,s);
     if multiline then
      begin
        if s=']' then
         begin
           multiline:=false;
           { overwrite last eol }
           dec(ptxt);
           ptxt^:=#0;
           inc(ptxt);
         end
        else if (s='') or (s[1] <> '#') then
         begin
           move(s[1],ptxt^,length(s));
           inc(ptxt,length(s));
           ptxt^:=#10;
           inc(ptxt);
         end;
      end
     else
      begin
        if (s<>'') and not(s[1] in ['#',';','%']) then
         begin
           i:=pos('=',s);
           if i>0 then
            begin
              j:=i+1;
              while (s[j] in ['0'..'9']) do
               inc(j);
              {enum}
              move(s[1],penum^,i-1);
              inc(penum,i-1);
              penum^:='=';
              inc(penum);
              number:=Copy(s,i+1,j-i-1);
              move(number[1],penum^,length(number));
              inc(penum,length(number));
              penum^:=#0;
              inc(penum);
              { multiline start then no txt }
              if s[j+1]='[' then
               begin
                 s1:=Copy(s,i+1,j-i);
                 move(s1[1],ptxt^,length(s1));
                 inc(ptxt,length(s1));
                 multiline:=true;
               end
              else
               begin
                 { txt including number }
                 s1:=Copy(s,i+1,255);
                 move(s1[1],ptxt^,length(s1));
                 inc(ptxt,length(s1));
                 ptxt^:=#0;
                 inc(ptxt);
               end;
            end;
         end;
      end;
   end;
  close(f);
end;


{*****************************************************************************
                               WriteEnumFile
*****************************************************************************}

procedure WriteEnumFile(const fn:string);
var
  t : text;
  i : longint;
  p : pchar;
  start : boolean;
begin
  writeln('Writing enumfile '+fn);
{Open textfile}
  assign(t,fn);
  rewrite(t);
  writeln(t,'const');
{Parse buffer in msgbuf and create indexs}
  p:=enumtxt;
  start:=true;
  for i:=1 to enumsize do
   begin
     if start then
      begin
        write(t,'  ');
        start:=false;
      end;
     if p^=#0 then
      begin
        writeln(t,';');
        start:=true;
      end
     else
      begin
        write(t,p^);
      end;
     inc(p);
   end;
  writeln(t,'');
  { msgtxt size }
  writeln(t,'  MsgTxtSize = ',msgsize,';');
  writeln(t,'');
  { max msg idx table }
  writeln(t,'  MsgIdxMax : array[1..20] of longint=(');
  write(t,'    ');
  for i:=1 to 20 do
   begin
     write(t,msgidxmax[i]+1);
     if i<20 then
      write(t,',');
     if i=10 then
      begin
        writeln(t,'');
        write(t,'    ');
      end;
   end;
  writeln(t,'');
  writeln(t,'  );');
  close(t);
end;


{*****************************************************************************
                               WriteStringFile
*****************************************************************************}

procedure WriteStringFile(const fn,constname:string);
const
  maxslen=240; { to overcome aligning problems }

  function l0(l:longint):string;
  var
    s : string[16];
  begin
    str(l,s);
    while (length(s)<5) do
     s:='0'+s;
    l0:=s;
  end;

var
  t      : text;
  f      : file;
  slen,
  len,i  : longint;
  p      : pchar;
  s      : string;
  start,
  quote  : boolean;
begin
  writeln('Writing stringfile ',fn);
{Open textfile}
  assign(t,fn);
  rewrite(t);
  writeln(t,'const '+constname+'_codepage=',msgcodepage:5,';');
  writeln(t,'{$ifdef Delphi}');
  writeln(t,'const '+constname+' : array[0..000000] of string[',maxslen,']=(');
  writeln(t,'{$else Delphi}');
  writeln(t,'const '+constname+' : array[0..000000,1..',maxslen,'] of char=(');
  write(t,'{$endif Delphi}');
{Parse buffer in msgbuf and create indexs}
  p:=msgtxt;
  slen:=0;
  len:=0;
  quote:=false;
  start:=true;
  for i:=1 to msgsize do
   begin
     if slen>=maxslen then
      begin
        if quote then
         begin
           write(t,'''');
           quote:=false;
         end;
        write(t,',');
        slen:=0;
        inc(len);
      end;
     if (len>70) or (start) then
      begin
        if quote then
         begin
           write(t,'''');
           quote:=false;
         end;
        if slen>0 then
          writeln(t,'+')
        else
          writeln(t);
        len:=0;
        start:=false;
      end;
     if (len=0) then
      write(t,'  ');
     if (ord(p^)>=32) and (p^<>#39) then
      begin
        if not quote then
         begin
           write(t,'''');
           quote:=true;
           inc(len);
         end;
        write(t,p^);
        inc(len);
      end
     else
      begin
        if quote then
         begin
           write(t,'''');
           inc(len);
           quote:=false;
         end;
        write(t,'#'+chr(ord(p^) div 100+48)+chr((ord(p^) mod 100) div 10+48)+chr(ord(p^) mod 10+48));
        inc(len,3);
      end;
     if p^ in [#0,#10] then
      start:=true;
     inc(slen);
     inc(p);
   end;
  if quote then
   write(t,'''');
  writeln(t,'');
  writeln(t,');');
  close(t);
{update arraysize}
  s:=l0((msgsize-1) div maxslen); { we start with 0 }
  assign(f,fn);
  reset(f,1);
  seek(f,22+34+2*eollen+2*length(constname));
  blockwrite(f,s[1],5);
  seek(f,22+90+4*eollen+3*length(constname));
  blockwrite(f,s[1],5);
  close(f);
end;


{*****************************************************************************
                               WriteCharFile
*****************************************************************************}

procedure WriteCharFile(const fn,constname:string);

  function l0(l:longint):string;
  var
    s : string[16];
  begin
    str(l,s);
    while (length(s)<5) do
     s:='0'+s;
    l0:=s;
  end;

  function createconst(b:byte):string;
  begin
    if (b in [32..127]) and (b<>39) then
     createconst:=''''+chr(b)+''''
    else
     createconst:='#'+chr(b div 100+48)+chr((b mod 100) div 10+48)+chr(b mod 10+48)
  end;

var
  t       : text;
  f       : file;
  cidx,i  : longint;
  p       : pchar;
  s       : string;
begin
  writeln('Writing charfile '+fn);
{Open textfile}
  assign(t,fn);
  rewrite(t);
  writeln(t,'const ',constname,' : array[1..00000] of char=(');
{Parse buffer in msgbuf and create indexs}
  p:=msgtxt;
  cidx:=0;
  for i:=1to msgsize do
   begin
     if cidx=15 then
      begin
        if cidx>0 then
         writeln(t,',')
        else
         writeln(t,'');
        write(t,'  ');
        cidx:=0;
      end
     else
      if cidx>0 then
        write(t,',')
      else
        write(t,'  ');
     write(t,createconst(ord(p^)));
     inc(cidx);
     inc(p);
   end;
  writeln(t,');');
  close(t);
{update arraysize}
  s:=l0(msgsize);
  assign(f,fn);
  reset(f,1);
  seek(f,18+length(constname));
  blockwrite(f,s[1],5);
  close(f);
end;


{*****************************************************************************
                               WriteIntelFile
*****************************************************************************}

procedure WriteIntelFile(const fn,constname:string);
var
  t      : text;
  len,i  : longint;
  p      : pchar;
  start,
  quote  : boolean;
begin
  writeln('Writing Intelfile ',fn);
{Open textfile}
  assign(t,fn);
  rewrite(t);
  writeln(t,'procedure '+constname+';assembler;');
  writeln(t,'asm');
{Parse buffer in msgbuf and create indexs}
  p:=msgtxt;
  len:=0;
  start:=true;
  quote:=false;
  for i:=1to msgsize do
   begin
     if len>70 then
      begin
        if quote then
         begin
           write(t,'''');
           quote:=false;
         end;
        writeln(t,'');
        start:=true;
      end;
     if start then
      begin
        write(t,'  db ''');
        len:=0;
        quote:=true;
      end;
     if (ord(p^)>=32) and (p^<>#39) then
      begin
        if not quote then
         begin
           write(t,',''');
           quote:=true;
           inc(len);
         end;
        write(t,p^);
        inc(len);
      end
     else
      begin
        if quote then
         begin
           write(t,'''');
           inc(len);
           quote:=false;
         end;
        write(t,','+chr(ord(p^) div 100+48)+chr((ord(p^) mod 100) div 10+48)+chr(ord(p^) mod 10+48));
        inc(len,4);
      end;
     inc(p);
   end;
  if quote then
   write(t,'''');
  writeln(t,'');
  writeln(t,'end;');
  close(t);
end;


{*****************************************************************************
                                RenumberFile
*****************************************************************************}

procedure RenumberFile(const fn,name:string);
var
  f,t  : text;
  i    : longint;
  s,s1 : string;
begin
  Writeln('Renumbering ',fn);
{Read the message file}
  assign(f,fn);
  {$push} {$I-}
   reset(f);
  {$pop}
  if ioresult<>0 then
   begin
     WriteLn('*** message file '+fn+' not found ***');
     exit;
   end;
  assign(t,'msg2inc.$$$');
  rewrite(t);
  i:=0;
  while not eof(f) do
   begin
     readln(f,s);
     if (copy(s,1,length(Name))=Name) and (s[3] in ['0'..'9']) then
      begin
        inc(i);
        str(i,s1);
        while length(s1)<3 do
         s1:='0'+s1;
        writeln(t,Name+s1+Copy(s,6,255));
      end
     else
      writeln(t,s);
   end;
  close(t);
  close(f);
{ rename new file }
  erase(f);
  rename(t,fn);
end;


{*****************************************************************************
                                WriteTexFile
*****************************************************************************}

Function EscapeString (Const S : String) : String;
Var
  I  : longint;
  hs : string;
begin
  hs:='';
  for i:=1 to length(s) do
    case S[i] of
      '$' :
        if (s[i+1] in ['0'..'9']) then
          hs:=hs+'arg'
        else
          hs:=hs+'\$';
      '&','{','}','#','_','%':            // Escape these characters
        hs := hs + '\' + S[i];
      '~','^':
        hs := hs + '\'+S[i]+' ';
      '\':
        hs:=hs+'$\backslash$'
    else
      hs := hs + S[i];
    end;
  EscapeString:=hs;
end;

procedure WriteTexFile(const infn,outfn:string);
var
  t,f   : text;
  line,
  i,k   : longint;
  s,s1  : string;
  texoutput : boolean;
begin
  Writeln('Loading messagefile ',infn);
  writeln('Writing TeXfile ',outfn);
{ Open infile }
  assign(f,infn);
  {$push} {$I-}
   reset(f);
  {$pop}
  if ioresult<>0 then
   begin
     WriteLn('*** message file '+infn+' not found ***');
     exit;
   end;
{ Open outfile }
  assign(t,outfn);
  rewrite(t);
  If texheader then
    begin
    writeln (t,'\documentclass{article}');
    writeln (t,'\usepackage{html}');
    writeln (t,'\usepackage{fpc}');
    writeln (t,'\begin{document}');
    end;
{ Parse }
  line:=0;
  TexOutput:=False;
  while not eof(f) do
   begin
     readln(f,s);
     inc(line);
     If Pos ('# BeginOfTeX',S)=1 then
       TexOutPut:=True
     else if pos ('# EndOfTeX',S)=1 then
       TexOutPut:=False;
     if (s<>'') and not(s[1] in ['#',';']) and TeXOutPut then
      begin
        if s[1]='%' then
         begin
           Delete(s,1,1);
           writeln(t,s);
         end
        else
         begin
           i:=pos('=',s);
           if i>0 then
            begin
              inc(i);
              while s[i] in ['0'..'9'] do
               inc(i);
              inc(i);
              s1:='';
              k:=0;
              while (k<5) and (s[i+k]<>'_') do
               begin
                 case s[i+k] of
                  'W' : s1:='Warning: ';
                  'E' : s1:='Error: ';
                  'F' : s1:='Fatal: ';
                  'N' : s1:='Note: ';
                  'I' : s1:='Info: ';
                  'H' : s1:='Hint: ';
                 end;
                 inc(k);
               end;
              if s[i+k]='_' then
               inc(i,k+1);
              writeln(t,'\item ['+s1+escapestring(Copy(s,i,255))+'] \hfill \\');
            end
           else
            writeln('error in line: ',line,' skipping');
         end;
      end;
   end;
  If TexHeader then
    writeln (t,'\end{document}');
  close(t);
  close(f);
end;


{*****************************************************************************
                                Main Program
*****************************************************************************}

procedure getpara;
var
  ch      : char;
  para    : string;
  files,i : word;

  procedure helpscreen;
  begin
    writeln('usage : msg2inc [Options] <msgfile> <incfile> <constname>');
    writeln('<Options> can be : -T     Create .doc TeX file');
    writeln('                   -TS    Create .doc TeX file (stand-alone)');
    writeln('                   -I     Intel style asm output');
    writeln('                   -S     array of string');
    writeln('                   -C     array of char');
    writeln('                   -R     renumber section <incfile>');
    writeln('                   -V     Show version');
    writeln('             -? or -H     This HelpScreen');
    halt(1);
  end;

begin
  Files:=0;
  for i:=1 to paramcount do
   begin
     para:=paramstr(i);
     if (para[1]='-') then
      begin
        ch:=upcase(para[2]);
        delete(para,1,2);
        case ch of
         'T' : begin
                 case upcase(para[1]) of
                  'S' : TexHeader:=True;
                 end;
                 Mode:=M_Tex;
               end;
         'I' : Mode:=M_Intel;
         'S' : Mode:=M_String;
         'C' : Mode:=M_Char;
         'R' : Mode:=M_Renumber;
         'V' : begin
                 Writeln('Msg2Inc ',version,' for Free Pascal (C) 1998-2002 Peter Vreman');
                 Writeln;
                 Halt;
               end;
     '?','H' : helpscreen;
        end;
     end
    else
     begin
       inc(Files);
       if Files>3 then
        HelpScreen;
       case Files of
        1 : InFile:=Para;
        2 : OutFile:=Para;
        3 : OutName:=Para;
       end;
     end;
   end;
  case Mode of
   M_Renumber,
        M_Tex : if Files<2 then
                 Helpscreen;
  else
   if Files<3 then
    HelpScreen;
  end;
end;


begin
  Mode:=M_String;
  OutFile:='';
  InFile:='';
  OutName:='';
  GetPara;
  case Mode of
   M_Renumber : begin
                  Renumberfile(Infile,OutFile);
                end;
        M_Tex : begin
                  WriteTexFile(InFile,Outfile);
                end;
      M_Intel : begin
                  Loadmsgfile(InFile);
                  WriteEnumFile(OutFile+'idx.inc');
                  WriteIntelFile(OutFile+'txt.inc',OutName+'txt');
                end;
     M_String : begin
                  Loadmsgfile(InFile);
                  WriteEnumFile(OutFile+'idx.inc');
                  WriteStringFile(OutFile+'txt.inc',OutName+'txt');
                end;
       M_Char : begin
                  Loadmsgfile(InFile);
                  WriteEnumFile(OutFile+'idx.inc');
                  WriteCharFile(OutFile+'txt.inc',OutName+'txt');
                end;
  end;
end.
