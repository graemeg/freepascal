{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Video unit for linux

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit video;

{$I-}
{$GOTO on}

{*****************************************************************************}
                                   interface
{*****************************************************************************}

{$i videoh.inc}

type  Tencoding=(cp437,         {Codepage 437}
                 cp850,         {Codepage 850}
                 cp852,         {Codepage 852}
                 cp866,         {Codepage 866}
                 koi8r,         {KOI8-R codepage}
                 iso01,         {ISO 8859-1}
                 iso02,         {ISO 8859-2}
                 iso03,         {ISO 8859-3}
                 iso04,         {ISO 8859-4}
                 iso05,         {ISO 8859-5}
                 iso06,         {ISO 8859-6}
                 iso07,         {ISO 8859-7}
                 iso08,         {ISO 8859-8}
                 iso09,         {ISO 8859-9}
                 iso10,         {ISO 8859-10}
                 iso13,         {ISO 8859-13}
                 iso14,         {ISO 8859-14}
                 iso15,         {ISO 8859-15}
                 utf8);         {UTF-8}

const  {Contains all code pages that can be considered a normal vga font.
        Note: KOI8-R has line drawing characters in wrong place. Support
              can perhaps be added, for now we'll let it rest.}
       vga_codepages=[cp437,cp850,cp852,cp866];
       iso_codepages=[iso01,iso02,iso03,iso04,iso05,iso06,iso07,iso08,
                      iso09,iso10,iso13,iso14,iso15];

var internal_codepage,external_codepage:Tencoding;


{*****************************************************************************}
                                implementation
{*****************************************************************************}

uses  baseunix,termio,strings
     {$ifdef linux},linuxvcs{$endif};

{$i video.inc}
{$i convert.inc}

type  Tconsole_type=(ttyNetwork
                     {$ifdef linux},ttyLinux{$endif}
                     ,ttyFreeBSD
                     ,ttyNetBSD);

      Tconversion=(cv_none,
                   cv_cp437_to_iso01,
                   cv_cp850_to_iso01,
                   cv_linuxlowascii_to_vga,
                   cv_cp437_to_UTF8);

      Ttermcode=(
        enter_alt_charset_mode,
        exit_alt_charset_mode,
        clear_screen,
        cursor_home,
        cursor_normal,
        cursor_visible_underline,
        cursor_visible_block,
        cursor_invisible,
        enter_ca_mode,
        exit_ca_mode,
        exit_am_mode,
        ena_acs
      );
      Ttermcodes=array[Ttermcode] of Pchar;
      Ptermcodes=^Ttermcodes;

const term_codes_ansi:Ttermcodes=
        (#$1B#$5B#$31#$31#$6D,                              {enter_alt_charset_mode}
         #$1B#$5B#$31#$30#$6D,                              {exit_alt_charset_mode}
         #$1B#$5B#$48#$1B#$5B#$4A,                          {clear_screen}
         #$1B#$5B#$48,                                      {cursor_home}
         nil,                                               {cursor_normal}
         nil,                                               {cursor visible, underline}
         nil,                                               {cursor visible, block}
         nil,                                               {cursor_invisible}
         nil,                                               {enter_ca_mode}
         nil,                                               {exit_ca_mode}
         nil,                                               {exit_am_mode}
         nil);                                              {ena_acs}

      term_codes_freebsd:Ttermcodes=
        (nil,                                               {enter_alt_charset_mode}
         nil,                                               {exit_alt_charset_mode}
         #$1B#$5B#$48#$1B#$5B#$4A,                          {clear_screen}
         #$1B#$5B#$48,                                      {cursor_home}
         #$1B#$5B#$3D#$30#$43,                              {cursor_normal}
         #$1B#$5B#$3D#$31#$43,                              {cursor visible, underline}
         #$1B#$5B#$3D#$31#$43,                              {cursor visible, block}
         nil,                                               {cursor_invisible}
         nil,                                               {enter_ca_mode}
         nil,                                               {exit_ca_mode}
         nil,                                               {exit_am_mode}
         nil);                                              {ena_acs}

      term_codes_linux:Ttermcodes=
        (#$1B#$5B#$31#$31#$6D,                              {enter_alt_charset_mode}
         #$1B#$5B#$31#$30#$6D,                              {exit_alt_charset_mode}
         #$1B#$5B#$48#$1B#$5B#$4A,                          {clear_screen}
         #$1B#$5B#$48,                                      {cursor_home}
         #$1B'[?25h'#$1B'[?0c',                             {cursor_normal}
         #$1B'[?0c',                                        {cursor visible, underline}
         #$1B'[?17;0;127c',                                 {cursor visible, block}
         #$1B'[?1c',                                        {cursor_invisible}
         nil,                                               {enter_ca_mode}
         nil,                                               {exit_ca_mode}
         nil,                                               {exit_am_mode}
         nil);                                              {ena_acs}

      term_codes_vt100:Ttermcodes=
        (#$0E,                                              {enter_alt_charset_mode}
         #$0F,                                              {exit_alt_charset_mode}
         #$1B#$5B#$48#$1B#$5B#$4A{#$24#$3C#$35#$30#$3E},    {clear_screen}
         #$1B#$5B#$48,                                      {cursor_home}
         nil,                                               {cursor_normal}
         nil,                                               {cursor visible, underline}
         nil,                                               {cursor visible, block}
         nil,                                               {cursor_invisible}
         nil,                                               {enter_ca_mode}
         nil,                                               {exit_ca_mode}
         #$1B#$5B#$3F#$37#$6C,                              {exit_am_mode}
         #$1B#$28#$42#$1B#$29#$30);                         {ena_acs}

      term_codes_vt220:Ttermcodes=
        (#$1B#$28#$30{#$24#$3C#$32#$3E},                    {enter_alt_charset_mode}
         #$1B#$28#$42{#$24#$3C#$34#$3E},                    {exit_alt_charset_mode}
         #$1B#$5B#$48#$1B#$5B#$4A,                          {clear_screen}
         #$1B#$5B#$48,                                      {cursor_home}
         nil,                                               {cursor_normal}
         nil,                                               {cursor visible, underline}
         nil,                                               {cursor visible, block}
         nil,                                               {cursor_invisible}
         nil,                                               {enter_ca_mode}
         nil,                                               {exit_ca_mode}
         #$1B#$5B#$3F#$37#$6C,                              {exit_am_mode}
         #$1B#$29#$30);                                     {ena_acs}

      term_codes_xterm:Ttermcodes=
        (#$0E,                                              {enter_alt_charset_mode}
         #$0F,                                              {exit_alt_charset_mode}
         #$1B#$5B#$48#$1B#$5B#$32#$4A,                      {clear_screen}
         #$1B#$5B#$48,                                      {cursor_home}
         #$1B#$5B#$3F#$31#$32#$6C#$1B#$5B#$3F#$32#$35#$68,  {cursor_normal}
         #$1B#$5B#$3F#$31#$32#$3B#$32#$35#$68,              {cursor visible, underline}
         #$1B#$5B#$3F#$31#$32#$3B#$32#$35#$68,              {cursor visible, block}
         #$1B#$5B#$3F#$32#$35#$6C,                          {cursor_invisible}
         #$1B#$5B#$3F#$31#$30#$34#$39#$68,                  {enter_ca_mode}
         #$1B#$5B#$3F#$31#$30#$34#$39#$6C,                  {exit_ca_mode}
         #$1B#$5B#$3F#$37#$6C,                              {exit_am_mode}
         #$1B#$28#$42#$1B#$29#$30);                         {ena_acs}

      term_codes_beos:Ttermcodes=
        (nil,//#$0E,                                              {enter_alt_charset_mode}
         nil,//#$0F,                                              {exit_alt_charset_mode}
         #$1B#$5B#$48#$1B#$5B#$4A,		                    {clear_screen}
         #$1B#$5B#$48,                                      {cursor_home}
         #$1B'[?25h',// nil,//#$1B#$5B#$3F#$31#$32#$6C#$1B#$5B#$3F#$32#$35#$68,  {cursor_normal}
         nil,//#$1B#$5B#$3F#$31#$32#$3B#$32#$35#$68,              {cursor visible, underline}
         nil,//#$1B#$5B#$3F#$31#$32#$3B#$32#$35#$68,              {cursor visible, block}
         #$1B'[?25l',//nil,//#$1B#$5B#$3F#$32#$35#$6C,                          {cursor_invisible}
         nil,//#$1B#$5B#$3F#$31#$30#$34#$39#$68,                  {enter_ca_mode}
         nil,//#$1B#$5B#$3F#$31#$30#$34#$39#$6C,                  {exit_ca_mode}
         nil,//#$1B#$5B#$3F#$37#$6C,                              {exit_am_mode}
         nil);//#$1B#$28#$42#$1B#$29#$30);                         {ena_acs}

const    terminal_names:array[0..11] of string[7]=(
                        'ansi',
                        'cons',
                        'eterm',
                        'gnome',
                        'konsole',
                        'linux',
                        'rxvt',
                        'screen',
                        'vt100',
                        'vt220',
                        'xterm',
                        'beterm');
         terminal_data:array[0..11] of Ptermcodes=(
                        @term_codes_ansi,
                        @term_codes_freebsd,
                        @term_codes_xterm,
                        @term_codes_xterm,
                        @term_codes_xterm,
                        @term_codes_linux,
                        @term_codes_xterm,
                        @term_codes_xterm,
                        @term_codes_vt100,
                        @term_codes_vt220,
                        @term_codes_xterm,
                        @term_codes_beos);

const convert:Tconversion=cv_none;

var
  LastCursorType : byte;
{$ifdef linux}
  TtyFd: Longint;
{$endif linux}
  Console: Tconsole_type;
  cur_term_strings:Ptermcodes;
{$ifdef logging}
  f: file;

const
  logstart: string = '';
  nl: char = #10;
  logend: string = #10#10;
{$endif logging}

{$ifdef cpui386}
{$ASMMODE ATT}
{$endif cpui386}

const

{  can_delete_term : boolean = false;}
  ACSIn : string = '';
  ACSOut : string = '';
  in_ACS : boolean =false;

  TerminalSupportsHighIntensityColors: boolean = false;
  TerminalSupportsBold: boolean = true;

function convert_vga_to_acs(ch:char):word;

{Ch contains a character in the VGA character set (i.e. codepage 437).
 This routine tries to convert some VGA symbols as well as possible to the
 xterm alternate character set.

 Return type is word to allow expanding to UCS-2 characters in the
 future.}

begin
  case ch of
    #18:
      convert_vga_to_acs:=word('|');
    #24, #30: {}
      convert_vga_to_acs:=word('^');
    #25, #31: {}
      convert_vga_to_acs:=word('v');
    #26, #16: {Never introduce a ctrl-Z ... }
      convert_vga_to_acs:=word('>');
    {#27,} #17: {}
      convert_vga_to_acs:=word('<');
    #176, #177, #178: {°±²}
      convert_vga_to_acs:=$f800+word('a');
    #180, #181, #182, #185: {´µ¶¹}
      convert_vga_to_acs:=$f800+word('u');
    #183, #184, #187, #191: {·¸»¿}
      convert_vga_to_acs:=$f800+word('k');
    #188, #189, #190, #217: {¼½¾Ù}
      convert_vga_to_acs:=$f800+word('j');
    #192, #200, #211, #212: {ÀÈÓÔ}
      convert_vga_to_acs:=$f800+word('m');
    #193, #202, #207, #208: {ÁÊÏÐ}
      convert_vga_to_acs:=$f800+word('v');
    #194, #203, #209, #210: {ÂËÑÒ}
      convert_vga_to_acs:=$f800+word('w');
    #195, #198, #199, #204: {ÃÆÇÌ}
      convert_vga_to_acs:=$f800+word('t');
    #196, #205: {ÄÍ}
      convert_vga_to_acs:=$f800+word('q');
    #179, #186: {³º}
      convert_vga_to_acs:=$f800+word('x');
    #197, #206, #215, #216: {ÅÎ×Ø}
      convert_vga_to_acs:=$f800+word('n');
    #201, #213, #214, #218: {ÉÕÖÚ}
      convert_vga_to_acs:=$f800+word('l');
    #254: { þ }
      convert_vga_to_acs:=word('*');
    { Shadows for Buttons }
    #220  { Ü },
    #223: { ß }
      convert_vga_to_acs:=$f800+word('a');
    else
      convert_vga_to_acs:=word(ch);
  end;
end;

procedure SendEscapeSeqNdx(ndx:Ttermcode);

var p:PChar;

begin
{ Always true because of vt100 default.
  if not assigned(cur_term_Strings) then
    exit}{RunError(219)};
  p:=cur_term_strings^[ndx];
  if p<>nil then
    fpwrite(stdoutputhandle,p^,strlen(p));
end;


procedure SendEscapeSeq(const S: String);
begin
  fpWrite(stdoutputhandle, S[1], Length(S));
end;


function IntStr(l:longint):string;

begin
  Str(l,intstr);
end;


Function XY2Ansi(x,y,ox,oy:longint):String;
{
  Returns a string with the escape sequences to go to X,Y on the screen.

  Note that x, y, ox, oy are 1-based (i.e. top-left corner of the screen
  is (1, 1)), while SetCursorPos parameters and CursorX and CursorY
  are 0-based (top-left corner of the screen is (0, 0)).
}

var delta:longint;
    direction:char;
    movement:string[32];

begin
  if ((x=1) and (oy+1=y)) and (console<>ttyfreebsd) then
    begin
      XY2Ansi:=#13#10;
      exit;
    end;
  direction:='H';
  if y=oy then
   begin
     if x=ox then
      begin
        XY2Ansi:='';
        exit;
      end;
     if x=1 then
      begin
        XY2Ansi:=#13;
        exit;
      end;
     delta:=ox-x;
     direction:=char(byte('C')+byte(x<=ox));
   end;
  if x=ox then
   begin
     delta:=oy-y;
     direction:=char(byte('A')+byte(y>oy));
   end;

  if direction='H' then
    movement:=intstr(y)+';'+intstr(x)
  else
    movement:=intstr(abs(delta));

  xy2ansi:=#27'['+movement+direction;
end;

const  ansitbl:array[0..7] of char='04261537';

{$ifdef disabled}
Function Attr2Ansi(Attr,OAttr:byte):string;
{
  Convert Attr to an Ansi String, the Optimal code is calculate
  with use of the old OAttr
}
var
  hstr : string[16];
  OFg,OBg,Fg,Bg:byte;

  procedure AddSep(ch:char);
  begin
    if length(hstr)>0 then
     hstr:=hstr+';';
    hstr:=hstr+ch;
  end;

begin
  if Attr=OAttr then
   begin
     Attr2Ansi:='';
     exit;
   end;
  Hstr:='';
  Fg:=Attr and $f;
  Bg:=Attr shr 4;
  OFg:=OAttr and $f;
  OBg:=OAttr shr 4;
{  This resets colours to their defaults, the problem is we don't know what
  the default is, i.e. it can either be white on black or back on white or
  even something totally different. This causes undesired colour schemes
  in the IDE on some terminals.
  if (OFg<>7) or (Fg=7) or ((OFg>7) and (Fg<8)) or ((OBg>7) and (Bg<8)) then
   begin
     hstr:='0';
     OFg:=7;
     OBg:=0;
   end;}
  if (Fg>7) and (OFg<8) then
   begin
     AddSep('1');
     OFg:=OFg or 8;
   end;
  if (Bg and 8)<>(OBg and 8) then
   begin
     AddSep('5');
     OBg:=OBg or 8;
   end;
  if (Fg<>OFg) then
   begin
     AddSep('3');
     hstr:=hstr+AnsiTbl[fg and 7];
   end;
  if (Bg<>OBg) then
   begin
     AddSep('4');
     hstr:=hstr+AnsiTbl[bg and 7];
   end;
  if hstr='0' then
   hstr:='';
  Attr2Ansi:=#27'['+hstr+'m';
end;
{$endif}

function attr2ansi(attr,oattr:byte):string;

var OFg,OBg,Fg,Bg:byte;

begin
  Fg:=Attr and $f;
  Bg:=Attr shr 4;
  OFg:=OAttr and $f;
  OBg:=OAttr shr 4;
  attr2ansi:=#27'[';
  if TerminalSupportsBold then
    if fg and 8<>0 then
      begin
        {Enable bold if not yet on.}
        if ofg and 8=0 then
          attr2ansi:=attr2ansi+'1;';
      end
    else
      {Disable bold if on.}
      if ofg and 8<>0 then
        attr2ansi:=attr2ansi+'22;';
  if bg and 8<>0 then
    begin
      {Enable bold if not yet on.}
      if obg and 8=0 then
        attr2ansi:=attr2ansi+'5;';
    end
  else
    {Disable bold if on.}
    if obg and 8<>0 then
      attr2ansi:=attr2ansi+'25;';

  if TerminalSupportsHighIntensityColors then
  begin
    if fg and 15<>ofg and 15 then
      if fg and 8<>0 then
        attr2ansi:=attr2ansi+'9'+ansitbl[fg and 7]+';'
      else
        attr2ansi:=attr2ansi+'3'+ansitbl[fg and 7]+';';
  end
  else
  begin
    if fg and 7<>ofg and 7 then
      attr2ansi:=attr2ansi+'3'+ansitbl[fg and 7]+';';
  end;
  if bg and 7<>obg and 7 then
     attr2ansi:=attr2ansi+'4'+ansitbl[bg and 7]+';';

  if attr2ansi[length(attr2ansi)]=';' then
    attr2ansi[length(attr2ansi)]:='m'
  else
   attr2ansi:='';
end;


procedure UpdateTTY(Force:boolean);
type
  tchattr=packed record
{$ifdef ENDIAN_LITTLE}
    ch : char;
    attr : byte;
{$else}
    attr : byte;
    ch : char;
{$endif}
  end;
var
  outbuf   : array[0..1023+255] of char;
  chattr   : tchattr;
  skipped  : boolean;
  outptr,
  spaces,
  eol,
  x,y,
  LastX,LastY,
  SpaceAttr,
  LastAttr : longint;
  p,pold   : pvideocell;
  LastLineWidth : Longint;

  function transform_cp437_to_iso01(const st:string):string;

  var i:byte;
      c:char;
      converted:word;

  begin
    transform_cp437_to_iso01:='';
    for i:=1 to length(st) do
      begin
        c:=st[i];
        case c of
          #0..#31:
            converted:=convert_lowascii_to_iso01[c];
          #128..#255:
            converted:=convert_cp437_to_iso01[c];
          else
            converted:=byte(c);
        end;
        if converted and $ff00=$f800 then
          begin
            if not in_ACS then
              begin
                transform_cp437_to_iso01:=transform_cp437_to_iso01+ACSIn;
                in_ACS:=true;
              end;
            c:=char(converted and $ff);
          end
        else
          if in_ACS then
            begin
              transform_cp437_to_iso01:=transform_cp437_to_iso01+ACSOut+
                                        Attr2Ansi(LastAttr,0);
              in_ACS:=false;
            end;
        transform_cp437_to_iso01:=transform_cp437_to_iso01+c;
      end;
  end;

  function transform_cp850_to_iso01(const st:string):string;

  var i:byte;
      c:char;
      converted:word;

  begin
    transform_cp850_to_iso01:='';
    for i:=1 to length(st) do
      begin
        c:=st[i];
        case c of
          #0..#31:
            converted:=convert_lowascii_to_iso01[c];
          #128..#255:
            converted:=convert_cp850_to_iso01[c];
          else
            converted:=byte(c);
        end;
        if converted and $ff00=$f800 then
          begin
            if not in_ACS then
              begin
                transform_cp850_to_iso01:=transform_cp850_to_iso01+ACSIn;
                in_ACS:=true;
              end;
          end
        else
          if in_ACS then
            begin
              transform_cp850_to_iso01:=transform_cp850_to_iso01+ACSOut+
                                        Attr2Ansi(LastAttr,0);
              in_ACS:=false;
            end;
        c:=char(converted and $ff);
        transform_cp850_to_iso01:=transform_cp850_to_iso01+c;
      end;
  end;

  function transform_linuxlowascii_to_vga(const st:string):string;

  var i:byte;
      c:char;
      converted:word;

  begin
    transform_linuxlowascii_to_vga:='';
    for i:=1 to length(st) do
      begin
        c:=st[i];
        case c of
          #0..#31:
            converted:=convert_linuxlowascii_to_vga[c];
          else
            converted:=byte(c);
        end;
        c:=char(converted and $ff);
        transform_linuxlowascii_to_vga:=transform_linuxlowascii_to_vga+c;
      end;
  end;

  function transform_cp437_to_UTF8(const st:string): string;
  var i:byte;
      c : char;
      converted : WideChar;
      s : WideString;
  begin
    transform_cp437_to_UTF8 := '';
    for i:=1 to length(st) do
      begin
        c:=st[i];
        case c of
          #0..#31:
            converted:=convert_lowascii_to_UTF8[c];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          #127..#255:
=======
          #128..#255:
>>>>>>> graemeg/fixes_2_2
=======
          #128..#255:
>>>>>>> origin/fixes_2_2
=======
          #127..#255:
>>>>>>> origin/fixes_2.4
            converted:=convert_cp437_to_UTF8[c];
          else
          begin
            converted := #0;
            converted := c;
          end;
        end;
        s := s + converted;
      end;
    transform_cp437_to_UTF8 := Utf8Encode(s);  
  end;
  
  function transform(const hstr:string):string;

  begin
    case convert of
      cv_linuxlowascii_to_vga:
        transform:=transform_linuxlowascii_to_vga(hstr);
      cv_cp437_to_iso01:
        transform:=transform_cp437_to_iso01(hstr);
      cv_cp850_to_iso01:
        transform:=transform_cp850_to_iso01(hstr);
      cv_cp437_to_UTF8:
      	transform:=transform_cp437_to_UTF8(hstr);
      else
        transform:=hstr;
    end;
  end;

  procedure outdata(hstr:string);

  begin
   If Length(HStr)>0 Then
   Begin
    while (eol>0) do
     begin
       hstr:=#13#10+hstr;
       dec(eol);
     end;
{    if (convert=cv_vga_to_acs) and (ACSIn<>'') and (ACSOut<>'') then
      transform_using_acs(Hstr);}
    move(hstr[1],outbuf[outptr],length(hstr));
    inc(outptr,length(hstr));
    if outptr>=1024 then
     begin
{$ifdef logging}
       blockwrite(f,logstart[1],length(logstart));
       blockwrite(f,nl,1);
       blockwrite(f,outptr,sizeof(outptr));
       blockwrite(f,nl,1);
       blockwrite(f,outbuf,outptr);
       blockwrite(f,nl,1);
{$endif logging}
       fpWrite(stdoutputhandle,outbuf,outptr);
       outptr:=0;
     end;
    end;
  end;

  procedure OutClr(c:byte);
  begin
    if c=LastAttr then
     exit;
    OutData(Attr2Ansi(c,LastAttr));
    LastAttr:=c;
  end;

  procedure OutSpaces;
  begin
    if (Spaces=0) then
     exit;
    OutClr(SpaceAttr);
    OutData(Space(Spaces));
    LastX:=x;
    LastY:=y;
    Spaces:=0;
  end;

(*
function GetTermString(ndx:Ttermcode):String;
var
   P{,pdelay}: PChar;
begin
  GetTermString:='';
  if not assigned(cur_term_Strings) then
    exit{RunError(219)};
  P:=cur_term_Strings^[Ndx];
  if assigned(p) then
   begin { Do not transmit the delays }
{     pdelay:=strpos(p,'$<');
     if assigned(pdelay) then
       pdelay^:=#0;}
     GetTermString:=StrPas(p);
{     if assigned(pdelay) then
       pdelay^:='$';}
   end;
end;
*)

begin
  OutPtr:=0;
  Eol:=0;
  skipped:=true;
  p:=PVideoCell(VideoBuf);
  pold:=PVideoCell(OldVideoBuf);
{ init Attr, X,Y and set autowrap off }
  SendEscapeSeq(#27'[0;40;37m'#27'[?7l'{#27'[H'} );
//  1.0.x: SendEscapeSeq(#27'[m'{#27'[H'});
  LastAttr:=7;
  LastX:=-1;
  LastY:=-1;
  for y:=1 to ScreenHeight do
   begin
     SpaceAttr:=0;
     Spaces:=0;
     LastLineWidth:=ScreenWidth;
     If (y=ScreenHeight) And (Console=ttyFreeBSD) {And :am: is on} Then
      LastLineWidth:=ScreenWidth-2;
     for x:=1 to LastLineWidth do
      begin
        if (not force) and (p^=pold^) then
         begin
           if (Spaces>0) then
            OutSpaces;
           skipped:=true;
         end
        else
         begin
           if skipped then
            begin
              OutData(XY2Ansi(x,y,LastX,LastY));
              LastX:=x;
              LastY:=y;
              skipped:=false;
            end;
           chattr:=tchattr(p^);
{           if chattr.ch in [#0,#255] then
            chattr.ch:=' ';}
           if chattr.ch=' ' then
            begin
              if Spaces=0 then
               SpaceAttr:=chattr.Attr;
              if (chattr.attr and $f0)=(spaceattr and $f0) then
               chattr.Attr:=SpaceAttr
              else
               begin
                 OutSpaces;
                 SpaceAttr:=chattr.Attr;
               end;
              inc(Spaces);
            end
           else
            begin
              if (Spaces>0) then
               OutSpaces;
{              if ord(chattr.ch)<32 then
                begin
                  Chattr.Attr:= $ff xor Chattr.Attr;
                  ChAttr.ch:=chr(ord(chattr.ch)+ord('A')-1);
                end;}
              if LastAttr<>chattr.Attr then
               OutClr(chattr.Attr);
              OutData(transform(chattr.ch));
              LastX:=x+1;
              LastY:=y;
            end;
           p^:=tvideocell(chattr);
         end;
        inc(p);
        inc(pold);
      end;
     if (Spaces>0) then
      OutSpaces;
     if force then
      inc(eol)
     else
      skipped:=true;
   end;
  eol:=0;
 {if am in capabilities? Then}
  if (Console=ttyFreeBSD) and (Plongint(p)^<>plongint(pold)^) Then
   begin
    OutData(XY2Ansi(ScreenWidth,ScreenHeight,LastX,LastY));
    OutData(#8);
    {Output last char}
    chattr:=tchattr(p[1]);
    if LastAttr<>chattr.Attr then
     OutClr(chattr.Attr);
    OutData(transform(chattr.ch));
    inc(LastX);
//    OutData(XY2Ansi(ScreenWidth-1,ScreenHeight,LastX,LastY));
//   OutData(GetTermString(Insert_character));
    OutData(#8+#27+'[1@');

    chattr:=tchattr(p^);
    if LastAttr<>chattr.Attr then
     OutClr(chattr.Attr);
    OutData(transform(chattr.ch));
    inc(LastX);
   end;
  OutData(XY2Ansi(CursorX+1,CursorY+1,LastX,LastY));
  if in_ACS then
    begin
      {If the program crashes and the ACS is still enabled, the user's
       keyboard will output strange characters. Therefore we disable the
       acs after each screen update, so the risk that it happens is greatly
       reduced.}
{      SendEscapeSeqNdx(exit_alt_charset_mode);}
      outdata(acsout);
      in_acs:=false;
    end;
{$ifdef logging}
  blockwrite(f,logstart[1],length(logstart));
  blockwrite(f,nl,1);
  blockwrite(f,outptr,sizeof(outptr));
  blockwrite(f,nl,1);
  blockwrite(f,outbuf,outptr);
  blockwrite(f,nl,1);
{$endif logging}
  fpWrite(stdoutputhandle,outbuf,outptr);
 {turn autowrap on}
//  SendEscapeSeq(#27'[?7h');
end;

{$ifdef linux}
procedure update_vcsa(force:boolean);

const max_updates=64;

label update,update_all,equal_loop,unequal_loop;

var position,update_count,i:word;
    update_positions:array[0..max_updates-1] of word;
    update_lengths:array[0..max_updates-1] of word;

begin
  if force then
    goto update_all;

  update_count:=0;
  i:=0;

equal_loop:
  repeat
    if videobuf^[i]<>oldvideobuf^[i] then
      goto unequal_loop;
    inc(i);
  until i>videobufsize div 2;
  goto update;

unequal_loop:
  if update_count>=max_updates then
    goto update_all;
  update_positions[update_count]:=i;
  update_lengths[update_count]:=0;
  inc(update_count);
  repeat
    if videobuf^[i]=oldvideobuf^[i] then
      goto equal_loop;
    inc(i);
    inc(update_lengths[update_count-1]);
  until i>videobufsize div 2;

update:
  for i:=1 to update_count do
    begin
      position:=update_positions[i-1];
      fppwrite(ttyfd,videobuf^[position],update_lengths[i-1]*2,4+position*2);
    end;
  exit;
update_all:
  fppwrite(ttyfd,videobuf^,videobufsize,4);
end;
{$endif}

var
  preInitVideoTio, postInitVideoTio: termio.termios;
  inputRaw, outputRaw: boolean;

procedure saveRawSettings(const tio: termio.termios);

begin
  with tio do
   begin
     inputRaw :=
       ((c_iflag and (IGNBRK or BRKINT or PARMRK or ISTRIP or
                                INLCR or IGNCR or ICRNL or IXON)) = 0) and
       ((c_lflag and (ECHO or ECHONL or ICANON or ISIG or IEXTEN)) = 0);
     outPutRaw :=
       ((c_oflag and OPOST) = 0) and
       ((c_cflag and (CSIZE or PARENB)) = 0) and
       ((c_cflag and CS8) <> 0);
   end;
end;

procedure restoreRawSettings(tio: termio.termios);
begin
  with tio do
    begin
      if inputRaw then
        begin
          c_iflag := c_iflag and (not (IGNBRK or BRKINT or PARMRK or ISTRIP or
            INLCR or IGNCR or ICRNL or IXON));
          c_lflag := c_lflag and
            (not (ECHO or ECHONL or ICANON or ISIG or IEXTEN));
          c_cc[VMIN]:=1;
          c_cc[VTIME]:=0;
        end;
     if outPutRaw then
       begin
         c_oflag := c_oflag and not(OPOST);
         c_cflag := c_cflag and not(CSIZE or PARENB) or CS8;
       end;
   end;
  TCSetAttr(1,TCSANOW,tio);
end;

function UTF8Enabled: Boolean;
var
  lang:string;
begin
  {$ifdef BEOS}
  UTF8Enabled := true;
  exit;
  {$endif}
  lang:=upcase(fpgetenv('LANG'));
  UTF8Enabled := (Pos('.UTF-8', lang) > 0) or (Pos('.UTF8', lang) > 0);
end;

procedure decide_codepages;

var s:string;

begin
  if external_codepage in vga_codepages then
    begin
      {Possible override...}
      s:=upcase(fpgetenv('CONSOLEFONT_CP'));
      if s='CP437' then
        external_codepage:=cp437
      else if s='CP850' then
        external_codepage:=cp850;
    end;
  {A non-vcsa Linux console can display most control characters, but not all.}
  if {$ifdef linux}(console<>ttyLinux) and{$endif}
     (cur_term_strings=@term_codes_linux) then
    convert:=cv_linuxlowascii_to_vga;
  case external_codepage of
    iso01:               {West Europe}
      begin
        internal_codepage:=cp850;
        convert:=cv_cp850_to_iso01;
      end;
    iso02:               {East Europe}
      internal_codepage:=cp852;
    iso05:               {Cyrillic}
      internal_codepage:=cp866;
    utf8:
      begin
        internal_codepage:=cp437;
        convert:=cv_cp437_to_UTF8;
      end;
    else
      if internal_codepage in vga_codepages then
        internal_codepage:=external_codepage
      else
        {We don't know how to convert to the external codepage. Use codepage
         437 in the hope that the actual font has similarity to codepage 437.}
        internal_codepage:=cp437;
  end;
end;


procedure prepareInitVideo;
begin
  TCGetAttr(1,preInitVideoTio);
  saveRawSettings(preInitVideoTio);
end;

procedure videoInitDone;
begin
  TCGetAttr(1,postInitVideoTio);
  restoreRawSettings(postInitVideoTio);
end;

procedure prepareDoneVideo;
var
  tio: termio.termios;
begin
  TCGetAttr(1,tio);
  saveRawSettings(tio);
  TCSetAttr(1,TCSANOW,postInitVideoTio);
end;

procedure doneVideoDone;
begin
  restoreRawSettings(preInitVideoTio);
end;

procedure SysInitVideo;
var
{$ifdef linux}
  FName: String;
{$endif linux}
  WS: packed record
    ws_row, ws_col, ws_xpixel, ws_ypixel: Word;
  end;
{  Err: Longint;}
{  prev_term : TerminalCommon_ptr1;}
  term:string;
  i:word;
{$ifdef Linux}
  s:string[15];
{$endif}
{$ifdef freebsd}
  ThisTTY: String[30];
{$endif}

const font_vga:array[0..11] of char=#15#27'%@'#27'(U'#27'[3h';
      font_lat1:array[0..5] of char=#27'%@'#27'(B';

begin
  { check for tty }
  if (IsATTY(stdinputhandle)=1) then
   begin
     { save current terminal characteristics and remove rawness }
     prepareInitVideo;
{$ifdef linux}
     { running on a tty, find out whether locally or remotely }
     TTyfd:=-1;
{$endif linux}
     Console:=TTyNetwork;                 {Default: Network or other vtxxx tty}
     cur_term_strings:=@term_codes_vt100; {Default: vt100}
     external_codepage:=iso01;            {Default: ISO-8859-1}
     if UTF8Enabled then
       external_codepage:=utf8;
   {$ifdef linux}
     if (vcs_device>=0) and (external_codepage<>utf8) then
       begin
         str(vcs_device,s);
         fname:='/dev/vcsa'+s;
         { open console, $1b6=rw-rw-rw- }
         ttyfd:=fpopen(fname,$1b6,O_RDWR);
         if ttyfd<>-1 then
           begin
             console:=ttylinux;
             external_codepage:=cp437;  {VCSA defaults to codepage 437.}
           end
         else
           if try_grab_vcsa then
             begin
               ttyfd:=fpopen(fname,$1b6,O_RDWR);
               if ttyfd<>-1 then
                 begin
                   console:=ttylinux;
                   external_codepage:=cp437;  {VCSA defaults to codepage 437.}
                 end;
             end;
       end;
   {$endif}
   {$ifdef freebsd}
     ThisTTY:=TTYName(stdinputhandle);
     if copy(ThisTTY, 1, 9) = '/dev/ttyv' then  {FreeBSD has these}
       begin
         { check for (Free?)BSD native}
         if (ThisTTY[10]>='0') and (ThisTTY[10]<='9') Then
            Console:=ttyFreeBSD;   {TTYFd ?}
       end;
   {$endif}
     term:=fpgetenv('TERM');
     for i:=low(terminal_names) to high(terminal_names) do
       if copy(term,1,length(terminal_names[i]))=terminal_names[i] then
         cur_term_strings:=terminal_data[i];
    if cur_term_strings=@term_codes_xterm then
    begin
<<<<<<< HEAD
      {$ifdef haiku}
      TerminalSupportsBold := true;
      TerminalSupportsHighIntensityColors := false;
      {$else}
      TerminalSupportsBold := false;
      TerminalSupportsHighIntensityColors := true;
      {$endif}
=======
      TerminalSupportsBold := false;
      TerminalSupportsHighIntensityColors := true;
>>>>>>> origin/fixes_2.4
    end
    else
    begin
      TerminalSupportsBold := true;
      TerminalSupportsHighIntensityColors := false;
    end;
<<<<<<< HEAD
    if cur_term_strings=@term_codes_beos then
    begin
      TerminalSupportsBold := false;
      TerminalSupportsHighIntensityColors := false;      
    end;  
=======
>>>>>>> origin/fixes_2.4
    if cur_term_strings=@term_codes_freebsd then
      console:=ttyFreeBSD;
{$ifdef linux}
    if (console<>ttylinux) then
      begin
{$endif}
        if cur_term_strings=@term_codes_linux then
          begin
            if external_codepage<>utf8 then
              begin
                {Enable the VGA character set (codepage 437,850,....)}
                fpwrite(stdoutputhandle,font_vga,sizeof(font_vga));
                external_codepage:=cp437;  {Now default to codepage 437.}
              end;
          end
        else
          begin
            if external_codepage<>utf8 then
              begin
                {No VGA font  :(  }
                fpwrite(stdoutputhandle,font_lat1,sizeof(font_lat1));
              end;
            { running on a remote terminal, no error with /dev/vcsa }
          end;
   {$ifdef linux}
      end;
   {$endif}
     fpioctl(stdinputhandle, TIOCGWINSZ, @WS);
     if WS.ws_Col=0 then
      WS.ws_Col:=80;
     if WS.ws_Row=0 then
      WS.ws_Row:=25;
     ScreenWidth:=WS.ws_Col;
     { TDrawBuffer only has FVMaxWidth elements
       larger values lead to crashes }
     if ScreenWidth> FVMaxWidth then
       ScreenWidth:=FVMaxWidth;
     ScreenHeight:=WS.ws_Row;
     CursorX:=0;
     CursorY:=0;
     LastCursorType:=$ff;
     ScreenColor:=True;
     { Start with a clear screen }
   {$ifdef linux}
     if Console<>ttylinux then
      begin
   {$endif}
        SendEscapeSeqNdx(cursor_home);
        SendEscapeSeqNdx(cursor_normal);
        SendEscapeSeqNdx(cursor_visible_underline);
        SendEscapeSeqNdx(enter_ca_mode);
        SetCursorType(crUnderLine);
        If Console=ttyFreeBSD Then
          SendEscapeSeqNdx(exit_am_mode);
   {$ifdef linux}
      end;
   {$endif}
{   Always true because of vt100 default...
      if assigned(cur_term_Strings) then
       begin}
         ACSIn:=StrPas(cur_term_strings^[enter_alt_charset_mode]);
         ACSOut:=StrPas(cur_term_strings^[exit_alt_charset_mode]);
         if (ACSIn<>'') and (ACSOut<>'') then
           SendEscapeSeqNdx(ena_acs);
(*         If fpGetEnv('TERM')='xterm' then
           convert:=cv_vga_to_acs;  {use of acs for xterm is ok}*)
{       end
     else
       begin
         ACSIn:='';
         ACSOut:='';
       end;}
{$ifdef logging}
     assign(f,'video.log');
     rewrite(f,1);
{$endif logging}
     { save new terminal characteristics and possible restore rawness }
     videoInitDone;

     decide_codepages;
   end
  else
   ErrorCode:=errVioInit; { not a TTY }
end;

procedure SysDoneVideo;

var font_custom:array[0..2] of char=#27'(K';

begin
  prepareDoneVideo;
  SetCursorType(crUnderLine);
{$ifdef linux}
  if Console=ttylinux then
   SetCursorPos(0,0)
  else
   begin
{$endif}
     SendEscapeSeqNdx(exit_ca_mode);
     SendEscapeSeqNdx(cursor_home);
     SendEscapeSeqNdx(cursor_normal);
     SendEscapeSeqNdx(cursor_visible_underline);
     SendEscapeSeq(#27'[H');
     if cur_term_strings=@term_codes_linux then
       begin
         {Executed in case ttylinux is false (i.e. no vcsa), but
          TERM=linux.}

         { if we're in utf8 mode, we didn't change the font, so
           no need to restore anything }
         if external_codepage<>utf8 then
         begin
           {Enable the character set set through setfont}
           fpwrite(stdoutputhandle,font_custom,3);
         end;
       end;
{$ifdef linux}
   end;
{$endif}
  ACSIn:='';
  ACSOut:='';
  doneVideoDone;
{$ifdef logging}
  close(f);
{$endif logging}
end;


procedure SysClearScreen;
begin
{$ifdef linux}
  if Console=ttylinux then
    UpdateScreen(true)
  else
    begin
{$endif}
      SendEscapeSeq(#27'[0m');
      SendEscapeSeqNdx(clear_screen);
{$ifdef linux}
    end;
{$endif}
end;


procedure SysUpdateScreen(Force: Boolean);
begin
{$ifdef linux}
  if console=ttylinux then
    update_vcsa(force)
  else
{$endif}
    updateTTY(force);
  move(VideoBuf^,OldVideoBuf^,VideoBufSize);
end;


function SysGetCapabilities: Word;
begin
{ about cpColor... we should check the terminfo database... }
  SysGetCapabilities:=cpUnderLine + cpBlink + cpColor;
end;


procedure SysSetCursorPos(NewCursorX, NewCursorY: Word);
{$ifdef linux}
var
  Pos : array [1..2] of Byte;
{$endif linux}
begin
 if (CursorX=NewCursorX) and (CursorY=NewCursorY) then
    exit;
{$ifdef linux}
  if Console=ttylinux then
   begin
     Pos[1]:=NewCursorX;
     Pos[2]:=NewCursorY;
     fppwrite(ttyfd,pos,2,2);
   end
  else
{$endif}
    { newcursorx,y and CursorX,Y are 0 based ! }
    SendEscapeSeq(XY2Ansi(NewCursorX+1,NewCursorY+1,CursorX+1,CursorY+1));
  CursorX:=NewCursorX;
  CursorY:=NewCursorY;
end;


function SysGetCursorType: Word;
begin
  SysGetCursorType:=LastCursorType;
end;


procedure SysSetCursorType(NewType: Word);
begin
  If LastCursorType=NewType then
   exit;
  LastCursorType:=NewType;
  case NewType of
    crBlock:
      SendEscapeSeqNdx(cursor_visible_block);
    crHidden:
      SendEscapeSeqNdx(cursor_invisible);
  else
    SendEscapeSeqNdx(cursor_normal);
  end;
end;

function SysSetVideoMode(const mode:Tvideomode):boolean;

var winsize:Twinsize;

begin
  {Due to xterm resize this procedure might get called with the new xterm
   size. Approve the video mode change if the new size equals that of
   the terminal window size.}
  SysSetVideoMode:=false;
  fpioctl(stdinputhandle,TIOCGWINSZ,@winsize);
  if (mode.row=winsize.ws_row) and
     (mode.col=winsize.ws_col) then
    begin
      screenwidth:=mode.col;
      screenheight:=mode.row;
      screencolor:=true;
      SysSetVideoMode:=true;
    end;
end;

Const
  SysVideoDriver : TVideoDriver = (
    InitDriver : @SysInitVideo;
    DoneDriver : @SysDoneVideo;
    UpdateScreen : @SysUpdateScreen;
    ClearScreen : @SysClearScreen;
    SetVideoMode : @SysSetVideoMode;
    GetVideoModeCount : Nil;
    GetVideoModeData : Nil;
    SetCursorPos : @SysSetCursorPos;
    GetCursorType : @SysGetCursorType;
    SetCursorType : @SysSetCursorType;
    GetCapabilities : @SysGetCapabilities;
  );

initialization
  SetVideoDriver(SysVideoDriver);
end.
