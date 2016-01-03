{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{***************************************************************************}
{                             Text output routines                          }
{***************************************************************************}

    const
       maxfonts    = 16;   { maximum possible fonts              }
       MaxChars    = 255;  { Maximum nr. of characters in a file }
       Prefix_Size = $80;  { prefix size to skip                 }
       SIGNATURE   = '+';  { Signature of CHR file               }

    type
      { Prefix header of Font file }
{      PFHeader = ^TFHeader;}
      TFHeader = packed record
         header_size: word;    {* Version 2.0 Header Format        *}
         font_name: array[1..4] of char;
         font_size: word;      {* Size in byte of file        *}
         font_major: byte;     {* Driver Version Information    *}
         font_minor: byte;
         min_major: byte;      {* BGI Revision Information         *}
         min_minor: byte;
      end;


      { Font record information }
{      PHeader = ^THeader;}
      THeader = packed record
        Signature:  char;     { signature byte                        }
        Nr_chars:   smallint;  { number of characters in file          }
        Reserved:   byte;
        First_char: byte;     { first character in file               }
        cdefs :     smallint;  { offset to character definitions       }
        scan_flag:  byte;     { TRUE if char is scanable              }
        org_to_cap: shortint;     { Height from origin to top of capitol  }
        org_to_base:shortint;     { Height from origin to baseline        }
        org_to_dec: shortint;     { Height from origin to bot of decender }
        _reserved: array[1..4] of char;
        Unused: byte;
      end;


      TOffsetTable =array[0..MaxChars] of smallint;
      TWidthTable  =array[0..MaxChars] of byte;

      tfontrec = packed record
        name : string[8];
        header : THeader;        { font header   }
        pheader : TFHeader;      { prefix header }
        offsets : TOffsetTable;
        widths : TWidthTable;
        instrlength: longint;    { length of instr, because instr can }
        instr : pchar;           { contain null characters            }
      end;



{      pStroke = ^TStroke;}
      TStroke = packed record
        opcode: byte;
        x: smallint;  { relative x offset character }
        y: smallint;  { relative y offset character }
      end;


      TStrokes = Array[0..1000] of TStroke;

      opcodes = (_END_OF_CHAR, _DO_SCAN, _DRAW := 253, _MOVE := 254 );


    var
       fonts : array[1..maxfonts] of tfontrec;
       Strokes: TStrokes; {* Stroke Data Base           *}
{       Stroke_count: Array[0..MaxChars] of smallint;} {* Stroke Count Table *}

{***************************************************************************}
{                         Internal support routines                         }
{***************************************************************************}

{$ifdef FPC_BIG_ENDIAN}
procedure swap_fheader(var h: tfheader);
(*
      TFHeader = packed record
         header_size: word;    {* Version 2.0 Header Format        *}
         font_name: array[1..4] of char;
         font_size: word;      {* Size in byte of file        *}
         font_major: byte;     {* Driver Version Information    *}
         font_minor: byte;
         min_major: byte;      {* BGI Revision Information         *}
         min_minor: byte;
      end;
*)
begin
  with h do
    begin
      header_size := swap(header_size);
      font_size := swap(font_size);
    end;
end;

procedure swap_header(var h: theader);
(*
      THeader = packed record
        Signature:  char;     { signature byte                        }
        Nr_chars:   smallint;  { number of characters in file          }
        Reserved:   byte;
        First_char: byte;     { first character in file               }
        cdefs :     smallint;  { offset to character definitions       }
        scan_flag:  byte;     { TRUE if char is scanable              }
        org_to_cap: shortint;     { Height from origin to top of capitol  }
        org_to_base:shortint;     { Height from origin to baseline        }
        org_to_dec: shortint;     { Height from origin to bot of decender }
        _reserved: array[1..4] of char;
        Unused: byte;
      end;
*)
begin
  with h do
    begin
      nr_chars := swap(nr_chars);
      cdefs := swap(cdefs);
    end;
end;


procedure swap_offsets(var t: toffsettable; start, len: longint);
(*
      TOffsetTable =array[0..MaxChars] of smallint;
*)
var
  i: longint;
begin
  for i := start to start+len-1 do
    t[i]:=Swap(t[i]);
end;
{$endif FPC_BIG_ENDIAN}


function ConvertString(const OrigString: String): String;
var
  i: Integer;
  ConvResult: String;
begin
  if GraphStringTransTable = nil then
    ConvertString := OrigString
  else
  begin
    SetLength(ConvResult, Length(OrigString));
    for i := 1 to Length(OrigString) do
      ConvResult[i] := GraphStringTransTable^[OrigString[i]];
    ConvertString := ConvResult;
  end;
end;


    function testfont(p : pchar) : boolean;

      begin
         testfont:=(p[0]='P') and
          (p[1]='K') and
          (p[2]=#8) and
          (p[3]=#8);
      end;


    function InstallUserFont(const FontFileName : string) : smallint;

      begin
         _graphresult:=grOk;
         { first check if we do not allocate too many fonts! }
         if installedfonts=maxfonts then
           begin
              _graphresult:=grError;
              InstallUserFont := DefaultFont;
              exit;
           end;
         inc(installedfonts);
         fonts[installedfonts].name:=FontFileName;
         fonts[installedfonts].instr := nil;
         fonts[installedfonts].instrlength := 0;
         InstallUserFont:=installedfonts;
      end;


    function Decode(byte1,byte2: char; var x,y: smallint): smallint;
    { This routines decoes a signle word in a font opcode section  }
    { to a stroke record.                                          }
      var
       b1,b2: shortint;
     Begin
       b1:=shortint(byte1);
       b2:=shortint(byte2);
       { Decode the CHR OPCODE }
       Decode:=byte((shortint(b1 and $80) shr 6)+(shortint(b2 and $80) shr 7));
       { Now get the X,Y coordinates        }
       { bit 0..7 only which are considered }
       { signed values.                     }
{ disable range check mode }
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$R-}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifopt R+}
{$define OPT_R_WAS_ON}
{$R-}
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
       b1:=b1 and $7f;
       b2:=b2 and $7f;
       { Now if the MSB of these values are set }
       { then the value is signed, therefore we }
       { sign extend it...                      }
       if (b1 and $40)<>0 then b1:=b1 or $80;
       if (b2 and $40)<>0 then b2:=b2 or $80;
       x:=smallint(b1);
       y:=smallint(b2);
{ restore previous range check mode }
<<<<<<< HEAD
<<<<<<< HEAD
{$pop}
=======
{$ifdef OPT_R_WAS_ON}
{$R+}
{$endif}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef OPT_R_WAS_ON}
{$R+}
{$endif}
>>>>>>> origin/fixes_2_2
     end;


    function unpack(buf: pchar; index: smallint; var Stroke: TStrokes): smallint;

     var
      po: TStrokes;
      num_ops: smallint;
      opcode, i, opc: word;
      counter: smallint;
      lindex: smallint;
      jx, jy: smallint;
     begin
       num_ops := 0;
       counter := index;
       lindex :=0;


       while TRUE do    {* For each byte in buffer      *}
         Begin
           Inc(num_ops);  {* Count the operation                *}
           opcode := decode( buf[counter], buf[counter+1] ,jx, jy );
           Inc(counter,2);
           if( opcode = ord(_END_OF_CHAR) ) then break; {* Exit loop at end of char     *}
         end;

       counter:=index;

       for i:=0 to num_ops-1 do    {    /* For each opcode in buffer    */ }
         Begin
           opc := decode(buf[counter], buf[counter+1], po[lindex].x, po[lindex].y);  {* Decode the data field   *}
           inc(counter,2);
           po[lindex].opcode := opc;      {* Save the opcode            *}
           Inc(lindex);
         end;
       Stroke:=po;
       unpack := num_ops;       {* return OPS count             *}
     end;



    procedure GetTextPosition(var xpos,ypos: longint; const TextString: string);
     begin
         if CurrentTextInfo.Font = DefaultFont then
          begin
           if Currenttextinfo.direction=horizdir then
            begin
              case Currenttextinfo.horiz of
                   centertext : XPos:=(textwidth(textstring) shr 1);
                   lefttext   : XPos:=0;
                   righttext  : XPos:=textwidth(textstring);
              end;
              case Currenttextinfo.vert of
                  centertext : YPos:=-(textheight(textstring) shr 1);
                  bottomtext : YPos:=-textheight(textstring);
                  toptext    : YPos:=0;
              end;
            end else
            begin
              case Currenttextinfo.horiz of
                   centertext : XPos:=(textheight(textstring) shr 1);
                   lefttext   : XPos:=textheight(textstring);
                   righttext  : XPos:=textheight(textstring);
              end;
              case Currenttextinfo.vert of
                  centertext : YPos:=(textwidth(textstring) shr 1);
                  bottomtext : YPos:=0;
                  toptext    : YPos:=textwidth(textstring);
              end;
            end;
          end
         else
          begin
            if Currenttextinfo.direction=horizdir then
            begin
              case CurrentTextInfo.horiz of
                   centertext : XPos:=(textwidth(textstring) shr 1);
                   lefttext   : XPos:=0;
                   righttext  : XPos:=textwidth(textstring);
              end;
              case CurrentTextInfo.vert of
                  centertext : YPos:=(textheight(textstring) shr 1);
                  bottomtext : YPos:=0;
                  toptext    : YPos:=textheight(textstring);
              end;
            end else
            begin
              case CurrentTextInfo.horiz of
                   centertext : XPos:=(textheight(textstring) shr 1);
                   lefttext   : XPos:=0;
                   righttext  : XPos:=textheight(textstring);
              end;
              case CurrentTextInfo.vert of
                  centertext : YPos:=(textwidth(textstring) shr 1);
                  bottomtext : YPos:=0;
                  toptext    : YPos:=textwidth(textstring);
              end;
           end;
          end;
     end;

{***************************************************************************}
{                         Exported routines                                 }
{***************************************************************************}


    function RegisterBGIfont(font : pointer) : smallint;

      var
         hp : pchar;
         b : word;
         i: longint;
         Header: THeader;
         counter: longint;
         FontData: pchar;
         FHeader: TFHeader;
      begin
         RegisterBGIfont:=grInvalidFontNum;
         i:=0;
         { Check if the font header is valid first of all }
         if testfont(font) then
           begin
              hp:=pchar(font);
              { Move to EOF in prefix header }
              while (hp[i] <> chr($1a)) do Inc(i);
              System.move(hp[i+1],FHeader,sizeof(FHeader));
              System.move(hp[Prefix_Size],header,sizeof(Header));
{$ifdef FPC_BIG_ENDIAN}
              swap_fheader(fheader);
              swap_header(header);
{$endif FPC_BIG_ENDIAN}
              { check if the font name is already allocated? }
              i:=Prefix_Size+sizeof(Header);
              for b:=1 to installedfonts do
                begin
                   if fonts[b].name=FHeader.Font_name then
                     begin
                        System.move(FHeader,fonts[b].PHeader,sizeof(FHeader));
                        System.move(Header,fonts[b].Header,sizeof(Header));
                        System.move(hp[i],Fonts[b].Offsets[Fonts[b].Header.First_Char],Fonts[b].Header.Nr_chars*sizeof(smallint));
{$ifdef FPC_BIG_ENDIAN}
                        swap_offsets(Fonts[b].Offsets,Fonts[b].Header.First_Char,Fonts[b].Header.Nr_chars);
{$endif FPC_BIG_ENDIAN}
                        Inc(i,Fonts[b].Header.Nr_chars*sizeof(smallint));
                        System.move(hp[i],Fonts[b].Widths[Fonts[b].Header.First_Char],Fonts[b].Header.Nr_chars*sizeof(byte));
                        Inc(i,Fonts[b].Header.Nr_chars*sizeof(byte));
                        counter:=Fonts[b].PHeader.font_size+PREFIX_SIZE-i;
                        { allocate also space for null }
                        GetMem(FontData,Counter+1);
                        System.move(hp[i],FontData^,Counter);
                        { Null terminate the string }
                        FontData[counter+1] := #0;
                        if fonts[b].header.Signature<> SIGNATURE then
                          begin
                            _graphResult:=grInvalidFont;
                            System.Freemem(FontData, Counter+1);
                            exit;
                          end;
                        fonts[b].instr:=FontData;
                        fonts[b].instrlength:=Counter+1;
                        RegisterBGIfont:=b;
                     end;
                end;
           end
         else
           RegisterBGIFont:=grInvalidFont;
      end;



    procedure GetTextSettings(var TextInfo : TextSettingsType);

      begin
         textinfo:=currenttextinfo;
      end;



    function TextHeight(const TextString : string) : word;

      begin
         if Currenttextinfo.font=DefaultFont
            then TextHeight:=8*CurrentTextInfo.CharSize
            else
              TextHeight:=Trunc((fonts[Currenttextinfo.font].header.org_to_cap-
                fonts[Currenttextinfo.font].header.org_to_dec) * CurrentYRatio);
      end;

    function TextWidth(const TextString : string) : word;
      var i,x : smallint;
          c   : byte;
          s   : String;
      begin
         x := 0;
         { if this is the default font ... }
         if Currenttextinfo.font = Defaultfont then
            TextWidth:=length(TextString)*8*CurrentTextInfo.CharSize
         { This is a stroked font ... }
            else begin
               s := ConvertString(TextString);
               for i:=1 to length(s) do
                begin
                   c:=byte(s[i]);
{                   dec(c,fonts[Currenttextinfo.font].header.first_char);}
                   if (c-fonts[Currenttextinfo.font].header.first_char>=
                       fonts[Currenttextinfo.font].header.nr_chars) then
                     continue;
                   x:=x+byte(fonts[Currenttextinfo.font].widths[c]);
               end;
             TextWidth:=round(x * CurrentXRatio) ;
            end;
      end;

    procedure OutTextXYDefault(x,y : smallint;const TextString : string);

      type
       Tpoint = record
         X,Y: smallint;
       end;
      var
         ConvString    : String;
         i,j,k,c       : longint;
         xpos,ypos     : longint;
         counter       : longint;
         cnt1,cnt2     : smallint;
         cnt3,cnt4     : smallint;
         charsize      : word;
         WriteMode     : word;
         curX2, curY2, xpos2, ypos2, x2, y2: graph_float;
         oldvalues     : linesettingstype;
         fontbitmap    : TBitmapChar;
<<<<<<< HEAD
<<<<<<< HEAD
         fontbitmapbyte: byte;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         chr           : char;
         curx2i,cury2i,
         xpos2i,ypos2i : longint;

      begin
         { save current write mode }
         WriteMode := CurrentWriteMode;
         CurrentWriteMode := NormalPut;
         GetTextPosition(xpos,ypos,textstring);
         X:=X-XPos; Y:=Y+YPos;
         XPos:=X; YPos:=Y;

         ConvString := ConvertString(TextString);
         CharSize := CurrentTextInfo.Charsize;
         if Currenttextinfo.font=DefaultFont then
         begin
           c:=length(ConvString);
           if CurrentTextInfo.direction=HorizDir then
           { Horizontal direction }
             begin
                for i:=0 to c-1 do
                  begin
                     xpos:=x+(i*8)*Charsize;
                     { we copy the character bitmap before accessing it }
                     { this improves speed on non optimizing compilers  }
                     { since it is one less address calculation.        }
                     Fontbitmap:=TBitmapChar(DefaultFontData[ConvString[i+1]]);
                     { no scaling }
                     if CharSize = 1 then
                      Begin
                        for j:=0 to 7 do
<<<<<<< HEAD
<<<<<<< HEAD
                          begin
                            fontbitmapbyte:=Fontbitmap[j];
                            for k:=0 to 7 do
                              begin
                                if (fontbitmapbyte and $80)<>0 then
                                  PutPixel(xpos+k,j+y,CurrentColor)
                                else if DrawTextBackground then
                                  PutPixel(xpos+k,j+y,CurrentBkColor);
                                fontbitmapbyte:=fontbitmapbyte shl 1;
                              end;
                          end;
=======
=======
>>>>>>> origin/fixes_2_2
                           for k:=0 to 7 do
                             if Fontbitmap[j,k]<>0 then
                               PutPixel(xpos+k,j+y,CurrentColor)
                             else if DrawTextBackground then
                               PutPixel(xpos+k,j+y,CurrentBkColor);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                      end
                     else
                      { perform scaling of bitmap font }
                      Begin
                        j:=0;
                        cnt3:=0;

                        while j <= 7 do
                        begin
                          { X-axis scaling }
                          for cnt4 := 0 to charsize-1 do
                           begin
                             k:=0;
                             cnt2 := 0;
<<<<<<< HEAD
<<<<<<< HEAD
                             fontbitmapbyte:=Fontbitmap[j];
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                             while k <= 7  do
                                begin
                                  for cnt1 := 0 to charsize-1 do
                                    begin
<<<<<<< HEAD
<<<<<<< HEAD
                                       If (fontbitmapbyte and $80) <> 0 then
=======
                                       If FontBitmap[j,k] <> 0 then
>>>>>>> graemeg/fixes_2_2
=======
                                       If FontBitmap[j,k] <> 0 then
>>>>>>> origin/fixes_2_2
                                           PutPixel(xpos+cnt1+cnt2,y+cnt3+cnt4,CurrentColor)
                                       else if DrawTextBackground then
                                           PutPixel(xpos+cnt1+cnt2,y+cnt3+cnt4,CurrentBkColor);
                                    end;
<<<<<<< HEAD
<<<<<<< HEAD
                                  Inc(k);
                                  Inc(cnt2,charsize);
                                  fontbitmapbyte:=fontbitmapbyte shl 1;
=======
                                    Inc(k);
                                    Inc(cnt2,charsize);
>>>>>>> graemeg/fixes_2_2
=======
                                    Inc(k);
                                    Inc(cnt2,charsize);
>>>>>>> origin/fixes_2_2
                                end;
                           end;
                          Inc(j);
                          Inc(cnt3,charsize);
                        end;
                      end;
                  end;
             end
           else
           { Vertical direction }
            begin
              for i:=0 to c-1 do
              begin

                chr := ConvString[i+1];
                Fontbitmap:=TBitmapChar(DefaultFontData[chr]);
                ypos := y-(i shl 3)*CharSize;

                { no scaling }
                if CharSize = 1 then
                 Begin
                   for j:=0 to 7 do
<<<<<<< HEAD
<<<<<<< HEAD
                     begin
                       fontbitmapbyte:=Fontbitmap[j];
                       for k:=0 to 7 do
                         begin
                           if (fontbitmapbyte and $80) <> 0 then
                             PutPixel(xpos+j,ypos-k,CurrentColor)
                           else if DrawTextBackground then
                             PutPixel(xpos+j,ypos-k,CurrentBkColor);
                           fontbitmapbyte:=fontbitmapbyte shl 1;
                         end;
                     end;
=======
=======
>>>>>>> origin/fixes_2_2
                      for k:=0 to 7 do
                        if Fontbitmap[j,k] <> 0 then
                          PutPixel(xpos+j,ypos-k,CurrentColor)
                        else if DrawTextBackground then
                          PutPixel(xpos+j,ypos-k,CurrentBkColor);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                 end
                else
                 { perform scaling of bitmap font }
                 Begin
                   j:=0;
                   cnt3:=0;

                   while j<=7 do
                   begin
                     { X-axis scaling }
                     for cnt4 := 0 to charsize-1 do
                      begin
                        k:=0;
                        cnt2 := 0;
<<<<<<< HEAD
<<<<<<< HEAD
                        fontbitmapbyte:=Fontbitmap[j];
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                        while k<=7  do
                           begin
                             for cnt1 := 0 to charsize-1 do
                               begin
<<<<<<< HEAD
<<<<<<< HEAD
                                  If (fontbitmapbyte and $80) <> 0 then
=======
                                  If FontBitmap[j,k] <> 0 then
>>>>>>> graemeg/fixes_2_2
=======
                                  If FontBitmap[j,k] <> 0 then
>>>>>>> origin/fixes_2_2
                                    PutPixel(xpos+cnt3-cnt4,ypos+cnt1-cnt2,CurrentColor)
                                  else if DrawTextBackground then
                                    PutPixel(xpos+cnt3-cnt4,ypos+cnt1-cnt2,CurrentBkColor);
                               end;
<<<<<<< HEAD
<<<<<<< HEAD
                             Inc(k);
                             Inc(cnt2,charsize);
                             fontbitmapbyte:=fontbitmapbyte shl 1;
=======
                               Inc(k);
                               Inc(cnt2,charsize);
>>>>>>> graemeg/fixes_2_2
=======
                               Inc(k);
                               Inc(cnt2,charsize);
>>>>>>> origin/fixes_2_2
                           end;
                      end;
                     Inc(j);
                     Inc(cnt3,charsize);
                   end;
                 end;
              end;
            end;
         end else
         { This is a stroked font which is already loaded into memory }
           begin
              getlinesettings(oldvalues);
              { reset line style to defaults }
              setlinestyle(solidln,oldvalues.pattern,normwidth);
              if Currenttextinfo.direction=vertdir then
                 xpos:=xpos + Textheight(ConvString);
              CurX2:=xpos; xpos2 := curX2; x2 := xpos2;
              CurY2:=ypos; ypos2 := curY2; y2 := ypos2;
{              x:=xpos; y:=ypos;}

              for i:=1 to length(ConvString) do
                begin
                   c:=byte(ConvString[i]);
{                   Stroke_Count[c] := }
                   unpack( fonts[CurrentTextInfo.font].instr,
                     fonts[CurrentTextInfo.font].Offsets[c], Strokes );
                   counter:=0;
                   while true do
                     begin
                         if CurrentTextInfo.direction=VertDir then
                           begin
                             xpos2:=x2-(Strokes[counter].Y*CurrentYRatio);
                             ypos2:=y2-(Strokes[counter].X*CurrentXRatio);
                           end
                         else
                           begin
                             xpos2:=x2+(Strokes[counter].X*CurrentXRatio) ;
                             ypos2:=y2-(Strokes[counter].Y*CurrentYRatio) ;
                           end;
                         case opcodes(Strokes[counter].opcode) of
                           _END_OF_CHAR: break;
                           _DO_SCAN: begin
                                    { Currently unsupported };
                                    end;
                           _MOVE : Begin
                                     CurX2 := XPos2;
                                     CurY2 := YPos2;
                                   end;
                           _DRAW: Begin
                                    curx2i:=trunc(CurX2);
                                    cury2i:=trunc(CurY2);
                                    xpos2i:=trunc(xpos2);
                                    ypos2i:=trunc(ypos2);
                                    { this optimization doesn't matter that much
                                    if (curx2i=xpos2i) then
                                      begin
                                         if (cury2i=ypos2i) then
                                           putpixel(curx2i,cury2i,currentcolor)
                                         else if (cury2i+1=ypos2i) or
                                           (cury2i=ypos2i+1) then
                                            begin
                                               putpixel(curx2i,cury2i,currentcolor);
                                               putpixel(curx2i,ypos2i,currentcolor);
                                            end
                                          else
                                            Line(curx2i,cury2i,xpos2i,ypos2i);
                                      end
                                    else if (cury2i=ypos2i) then
                                      begin
                                         if (curx2i+1=xpos2i) or
                                           (curx2i=xpos2i+1) then
                                            begin
                                               putpixel(curx2i,cury2i,currentcolor);
                                               putpixel(xpos2i,cury2i,currentcolor);
                                            end
                                          else
                                            Line(curx2i,cury2i,xpos2i,ypos2i);
                                      end
                                    else
                                    }
                                    Line(curx2i,cury2i,xpos2i,ypos2i);
                                    CurX2:=xpos2;
                                    CurY2:=ypos2;
                                  end;
                             else
                               Begin
                               end;
                            end;
                        Inc(counter);
                     end; { end while }
                   if Currenttextinfo.direction=VertDir then
                     y2:=y2-(byte(fonts[CurrenttextInfo.font].widths[c])*CurrentXRatio)
                   else
                     x2:=x2+(byte(fonts[Currenttextinfo.font].widths[c])*CurrentXRatio);
                end;
              setlinestyle( oldvalues.linestyle, oldvalues.pattern, oldvalues.thickness);
           end;
        { restore write mode }
        CurrentWriteMode := WriteMode;
      end;


    procedure OutText(const TextString : string);
      var x,y:smallint;
      begin
         { Save CP }
         x:=CurrentX;
         y:=CurrentY;
         OutTextXY(CurrentX,CurrentY,TextString);
         { If the direction is Horizontal and the justification left }
         { then and only then do we update the CP                    }
         if (Currenttextinfo.direction=HorizDir) and
           (Currenttextinfo.horiz=LeftText) then
               inc(x,textwidth(TextString));
         { Update the CP }
         CurrentX := X;
         CurrentY := Y;
      end;





    procedure SetTextJustify(horiz,vert : word);

      begin
         if (horiz<0) or (horiz>2) or
            (vert<0) or (vert>2) then
           begin
              _graphresult:=grError;
              exit;
           end;
         Currenttextinfo.horiz:=horiz;
         Currenttextinfo.vert:=vert;
      end;


    procedure SetTextStyle(font,direction : word;charsize : word);

      var
         f : file;
         Prefix: array[0..Prefix_Size-1] of char; {* File Prefix Holder         *}
         Length, Current: longint;
         FontData: Pchar;
         hp  : pchar;
         i   : longint;
      begin
         if font>installedfonts then
           begin
              _graphresult:=grInvalidFontNum;
              exit;
           end;

         Currenttextinfo.font:=font;
         if (direction<>HorizDir) and (direction<>VertDir) then
           direction:=HorizDir;
         Currenttextinfo.direction:=direction;
         { According to the Turbo Pascal programmer's reference }
         { maximum charsize for bitmapped font is 10            }
         if (CurrentTextInfo.Font = DefaultFont) and (Charsize > 10) then
            Currenttextinfo.charsize:=10
         else if charsize<1 then
            Currenttextinfo.charsize:=1
         else
            Currenttextinfo.charsize:=charsize;

         { This is only valid for stroked fonts }
{$ifdef logging}
         LogLn('(org_to_cap - org_to_dec): ' + strf(
                fonts[Currenttextinfo.font].header.org_to_cap-
                fonts[Currenttextinfo.font].header.org_to_dec));
{$endif logging}
         if (charsize <> usercharsize) then
           Case CharSize of
             1: Begin
                  CurrentXRatio := 0.55;
                  CurrentYRatio := 0.55;
                End;
             2: Begin
                  CurrentXRatio := 0.65;
                  CurrentYRatio := 0.65;
                End;
             3: Begin
                  CurrentXRatio := 0.75;
                  CurrentYRatio := 0.75;
                End;
             4: Begin
                  CurrentXRatio := 1.0;
                  CurrentYRatio := 1.0;
                End;
             5: Begin
                  CurrentXRatio := 1.3;
                  CurrentYRatio := 1.3;
                End;
             6: Begin
                  CurrentXRatio := 1.65;
                  CurrentYRatio := 1.65
                End;
             7: Begin
                  CurrentXRatio := 2.0;
                  CurrentYRatio := 2.0;
                End;
             8: Begin
                  CurrentXRatio := 2.5;
                  CurrentYRatio := 2.5;
                End;
             9: Begin
                  CurrentXRatio := 3.0;
                  CurrentYRatio := 3.0;
                End;
             10: Begin
                   CurrentXRatio := 4.0;
                   CurrentYRatio := 4.0;
                 End
           End;
         { if this is a stroked font then load it if not already loaded }
         { into memory...                                               }
         if (font>DefaultFont) and not assigned(fonts[font].instr) then
           begin
              assign(f,bgipath+fonts[font].name+'.CHR');
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$i-}
              reset(f,1);
{$pop}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifopt I+}
{$define IOCHECK_WAS_ON}
{$i-}
{$endif}
              reset(f,1);
{$ifdef IOCHECK_WAS_ON}
{$i+}
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
              if ioresult<>0 then
                begin
                   _graphresult:=grFontNotFound;
                   Currenttextinfo.font:=DefaultFont;
                   exit;
                end;
              {* Read in the file prefix        *}
              BlockRead(F, Prefix, Prefix_Size);
              hp:=Prefix;
              i:=0;
              while (hp[i] <> chr($1a)) do Inc(i);
              System.move(hp[i+1],fonts[font].PHeader,sizeof(TFHeader));
              (* Read in the Header file  *)
              BlockRead(F,fonts[font].Header,Sizeof(THeader));
{$ifdef FPC_BIG_ENDIAN}
              swap_fheader(fonts[font].PHeader);
              swap_header(fonts[font].Header);
{$endif FPC_BIG_ENDIAN}
              BlockRead(F,Fonts[font].Offsets[Fonts[font].Header.First_Char],Fonts[font].Header.Nr_chars*sizeof(smallint));
{$ifdef FPC_BIG_ENDIAN}
              swap_offsets(Fonts[font].Offsets,Fonts[font].Header.First_Char,Fonts[font].Header.Nr_chars);
{$endif FPC_BIG_ENDIAN}
              {*        Load the character width table into memory.                     *}
              BlockRead(F,Fonts[font].Widths[Fonts[font].Header.First_Char],Fonts[font].Header.Nr_chars*sizeof(byte));
              {*        Determine the length of the stroke database.                    *}
              current := FilePos( f );          {* Current file location        *}
              Seek( f, FileSize(F));            {* Go to the end of the file    *}
              length := FilePos( f );           {* Get the file length          *}
              Seek( f, current);        {* Restore old file location    *}
              {*        Load the stroke database.                                       *}
              { also allocate space for Null character   }
              Getmem(FontData, Length+1);          {* Create space for font data        *}

              BlockRead(F, FontData^, length-current);        {* Load the stroke data   *}
              FontData[length-current+1] := #0;

             if fonts[font].header.Signature<> SIGNATURE then
             begin
                _graphResult:=grInvalidFont;
                Currenttextinfo.font:=DefaultFont;
                System.Freemem(FontData, Length+1);
                exit;
             end;
             fonts[font].instr:=FontData;
             fonts[font].instrLength:=Length+1;


              if not testfont(Prefix) then
                begin
                   _graphresult:=grInvalidFont;
                   Currenttextinfo.font:=DefaultFont;
                   System.Freemem(FontData, Length+1);
                end;
              close(f);
           end;
      end;

    procedure SetUserCharSize(Multx,Divx,Multy,Divy : word);
      begin
         CurrentXRatio := MultX / DivX;
         CurrentYRatio := MultY / DivY;
      end;

