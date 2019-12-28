{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000 by Florian Klaempfl
    member of the Free Pascal development team.

    This unit implements several classes for charset conversions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
unit ccharset;

{$i fpcdefs.inc}

  interface

    type
       tunicodechar = word;
       tunicodestring = ^tunicodechar;

       tcsconvert = class
       end;

       tunicodecharmappingflag = (umf_noinfo,umf_leadbyte,umf_undefined,
         umf_unused);

       punicodecharmapping = ^tunicodecharmapping;
       tunicodecharmapping = record
          unicode : tunicodechar;
          flag : tunicodecharmappingflag;
          reserved : byte;
       end;

       punicodemap = ^tunicodemap;
       tunicodemap = record
          cpname : string[20];
          cp : word;
          map : punicodecharmapping;
          lastchar : longint;
          next : punicodemap;
          internalmap : boolean;
       end;

       tcp2unicode = class(tcsconvert)
       end;

    const
       DefaultSystemCodePage = 437;

    function loadunicodemapping(const cpname,f : string; cp :word) : punicodemap;
    procedure registermapping(p : punicodemap);
    function getmap(const s : string) : punicodemap;
    function getmap(cp : word) : punicodemap;
    function mappingavailable(const s : string) : boolean;
    function mappingavailable(cp :word) : boolean;
    function getunicode(c : char;p : punicodemap) : tunicodechar;
    function getascii(c : tunicodechar;p : punicodemap) : string;

  implementation

    var
       mappings : punicodemap;

    function loadunicodemapping(const cpname,f : string; cp :word) : punicodemap;

      var
         data : punicodecharmapping;
         datasize : longint;
         t : text;
         s,hs : string;
         scanpos,charpos,unicodevalue : longint;
         code : word;
         flag : tunicodecharmappingflag;
         p : punicodemap;
         lastchar : longint;

      begin
         lastchar:=-1;
         loadunicodemapping:=nil;
         datasize:=256;
         getmem(data,sizeof(tunicodecharmapping)*datasize);
         assign(t,f);
         {$I-}
         reset(t);
         {$I+}
         if ioresult<>0 then
           begin
              freemem(data,sizeof(tunicodecharmapping)*datasize);
              exit;
           end;
         while not(eof(t)) do
           begin
              readln(t,s);
              if (s[1]='0') and (s[2]='x') then
                begin
                   flag:=umf_unused;
                   scanpos:=3;
                   hs:='$';
                   while s[scanpos] in ['0'..'9','A'..'F','a'..'f'] do
                     begin
                        hs:=hs+s[scanpos];
                        inc(scanpos);
                     end;
                   val(hs,charpos,code);
                   if code<>0 then
                     begin
                        freemem(data,sizeof(tunicodecharmapping)*datasize);
                        close(t);
                        exit;
                     end;
                   while not(s[scanpos] in ['0','#']) do
                     inc(scanpos);
                   if s[scanpos]='#' then
                     begin
                        { special char }
                        unicodevalue:=$ffff;
                        hs:=copy(s,scanpos,length(s)-scanpos+1);
                        if hs='#DBCS LEAD BYTE' then
                          flag:=umf_leadbyte;
                     end
                   else
                     begin
                        { C hex prefix }
                        inc(scanpos,2);
                        hs:='$';
                        while s[scanpos] in ['0'..'9','A'..'F','a'..'f'] do
                          begin
                             hs:=hs+s[scanpos];
                             inc(scanpos);
                          end;
                        val(hs,unicodevalue,code);
                        if code<>0 then
                          begin
                             freemem(data,sizeof(tunicodecharmapping)*datasize);
                             close(t);
                             exit;
                          end;
                        if charpos>datasize then
                          begin
                             { allocate 1024 bytes more because         }
                             { if we need more than 256 entries it's    }
                             { probably a mbcs with a lot of            }
                             { entries                                  }
                             datasize:=charpos+1024;
                             reallocmem(data,sizeof(tunicodecharmapping)*datasize);
                          end;
                        flag:=umf_noinfo;
                     end;
                   data[charpos].flag:=flag;
                   data[charpos].unicode:=unicodevalue;
                   if charpos>lastchar then
                     lastchar:=charpos;
                end;
           end;
         close(t);
         new(p);
         p^.lastchar:=lastchar;
         p^.cpname:=cpname;
         p^.cp:=cp;
         p^.internalmap:=false;
         p^.next:=nil;
         p^.map:=data;
         loadunicodemapping:=p;
      end;

    procedure registermapping(p : punicodemap);

      begin
         p^.next:=mappings;
         mappings:=p;
      end;

    function getmap(const s : string) : punicodemap;

      var
         hp : punicodemap;

      const
         mapcache : string = '';
         mapcachep : punicodemap = nil;

      begin
         if (mapcache=s) and assigned(mapcachep) and (mapcachep^.cpname=s) then
           begin
              getmap:=mapcachep;
              exit;
           end;
         hp:=mappings;
         while assigned(hp) do
           begin
              if hp^.cpname=s then
                begin
                   getmap:=hp;
                   mapcache:=s;
                   mapcachep:=hp;
                   exit;
                end;
              hp:=hp^.next;
           end;
         getmap:=nil;
      end;

    function getmap(cp : word) : punicodemap;

      var
         hp : punicodemap;

      const
         mapcache : word = 0;
         mapcachep : punicodemap = nil;

      begin
         if (mapcache=cp) and assigned(mapcachep) and (mapcachep^.cp=cp) then
           begin
              getmap:=mapcachep;
              exit;
           end;
         hp:=mappings;
         while assigned(hp) do
           begin
              if hp^.cp=cp then
                begin
                   getmap:=hp;
                   mapcache:=cp;
                   mapcachep:=hp;
                   exit;
                end;
              hp:=hp^.next;
           end;
         getmap:=nil;
      end;

    function mappingavailable(const s : string) : boolean;

      begin
         mappingavailable:=getmap(s)<>nil;
      end;

    function mappingavailable(cp : word) : boolean;

      begin
         mappingavailable:=getmap(cp)<>nil;
      end;

    function getunicode(c : char;p : punicodemap) : tunicodechar;

      begin
         if ord(c)<=p^.lastchar then
           getunicode:=p^.map[ord(c)].unicode
         else
           getunicode:=0;
      end;

    function getascii(c : tunicodechar;p : punicodemap) : string;

      var
         i : longint;

      begin
         { at least map to '?' }
         getascii:=#63;
         for i:=0 to p^.lastchar do
           if p^.map[i].unicode=c then
             begin
                if i<256 then
                  getascii:=chr(i)
                else
                  getascii:=chr(i div 256)+chr(i mod 256);
                exit;
             end;
      end;

   const
     map : array[0..255] of tunicodecharmapping = (
       (unicode : 0; flag : umf_noinfo; reserved: 0),
       (unicode : 1; flag : umf_noinfo; reserved: 0),
       (unicode : 2; flag : umf_noinfo; reserved: 0),
       (unicode : 3; flag : umf_noinfo; reserved: 0),
       (unicode : 4; flag : umf_noinfo; reserved: 0),
       (unicode : 5; flag : umf_noinfo; reserved: 0),
       (unicode : 6; flag : umf_noinfo; reserved: 0),
       (unicode : 7; flag : umf_noinfo; reserved: 0),
       (unicode : 8; flag : umf_noinfo; reserved: 0),
       (unicode : 9; flag : umf_noinfo; reserved: 0),
       (unicode : 10; flag : umf_noinfo; reserved: 0),
       (unicode : 11; flag : umf_noinfo; reserved: 0),
       (unicode : 12; flag : umf_noinfo; reserved: 0),
       (unicode : 13; flag : umf_noinfo; reserved: 0),
       (unicode : 14; flag : umf_noinfo; reserved: 0),
       (unicode : 15; flag : umf_noinfo; reserved: 0),
       (unicode : 16; flag : umf_noinfo; reserved: 0),
       (unicode : 17; flag : umf_noinfo; reserved: 0),
       (unicode : 18; flag : umf_noinfo; reserved: 0),
       (unicode : 19; flag : umf_noinfo; reserved: 0),
       (unicode : 20; flag : umf_noinfo; reserved: 0),
       (unicode : 21; flag : umf_noinfo; reserved: 0),
       (unicode : 22; flag : umf_noinfo; reserved: 0),
       (unicode : 23; flag : umf_noinfo; reserved: 0),
       (unicode : 24; flag : umf_noinfo; reserved: 0),
       (unicode : 25; flag : umf_noinfo; reserved: 0),
       (unicode : 26; flag : umf_noinfo; reserved: 0),
       (unicode : 27; flag : umf_noinfo; reserved: 0),
       (unicode : 28; flag : umf_noinfo; reserved: 0),
       (unicode : 29; flag : umf_noinfo; reserved: 0),
       (unicode : 30; flag : umf_noinfo; reserved: 0),
       (unicode : 31; flag : umf_noinfo; reserved: 0),
       (unicode : 32; flag : umf_noinfo; reserved: 0),
       (unicode : 33; flag : umf_noinfo; reserved: 0),
       (unicode : 34; flag : umf_noinfo; reserved: 0),
       (unicode : 35; flag : umf_noinfo; reserved: 0),
       (unicode : 36; flag : umf_noinfo; reserved: 0),
       (unicode : 37; flag : umf_noinfo; reserved: 0),
       (unicode : 38; flag : umf_noinfo; reserved: 0),
       (unicode : 39; flag : umf_noinfo; reserved: 0),
       (unicode : 40; flag : umf_noinfo; reserved: 0),
       (unicode : 41; flag : umf_noinfo; reserved: 0),
       (unicode : 42; flag : umf_noinfo; reserved: 0),
       (unicode : 43; flag : umf_noinfo; reserved: 0),
       (unicode : 44; flag : umf_noinfo; reserved: 0),
       (unicode : 45; flag : umf_noinfo; reserved: 0),
       (unicode : 46; flag : umf_noinfo; reserved: 0),
       (unicode : 47; flag : umf_noinfo; reserved: 0),
       (unicode : 48; flag : umf_noinfo; reserved: 0),
       (unicode : 49; flag : umf_noinfo; reserved: 0),
       (unicode : 50; flag : umf_noinfo; reserved: 0),
       (unicode : 51; flag : umf_noinfo; reserved: 0),
       (unicode : 52; flag : umf_noinfo; reserved: 0),
       (unicode : 53; flag : umf_noinfo; reserved: 0),
       (unicode : 54; flag : umf_noinfo; reserved: 0),
       (unicode : 55; flag : umf_noinfo; reserved: 0),
       (unicode : 56; flag : umf_noinfo; reserved: 0),
       (unicode : 57; flag : umf_noinfo; reserved: 0),
       (unicode : 58; flag : umf_noinfo; reserved: 0),
       (unicode : 59; flag : umf_noinfo; reserved: 0),
       (unicode : 60; flag : umf_noinfo; reserved: 0),
       (unicode : 61; flag : umf_noinfo; reserved: 0),
       (unicode : 62; flag : umf_noinfo; reserved: 0),
       (unicode : 63; flag : umf_noinfo; reserved: 0),
       (unicode : 64; flag : umf_noinfo; reserved: 0),
       (unicode : 65; flag : umf_noinfo; reserved: 0),
       (unicode : 66; flag : umf_noinfo; reserved: 0),
       (unicode : 67; flag : umf_noinfo; reserved: 0),
       (unicode : 68; flag : umf_noinfo; reserved: 0),
       (unicode : 69; flag : umf_noinfo; reserved: 0),
       (unicode : 70; flag : umf_noinfo; reserved: 0),
       (unicode : 71; flag : umf_noinfo; reserved: 0),
       (unicode : 72; flag : umf_noinfo; reserved: 0),
       (unicode : 73; flag : umf_noinfo; reserved: 0),
       (unicode : 74; flag : umf_noinfo; reserved: 0),
       (unicode : 75; flag : umf_noinfo; reserved: 0),
       (unicode : 76; flag : umf_noinfo; reserved: 0),
       (unicode : 77; flag : umf_noinfo; reserved: 0),
       (unicode : 78; flag : umf_noinfo; reserved: 0),
       (unicode : 79; flag : umf_noinfo; reserved: 0),
       (unicode : 80; flag : umf_noinfo; reserved: 0),
       (unicode : 81; flag : umf_noinfo; reserved: 0),
       (unicode : 82; flag : umf_noinfo; reserved: 0),
       (unicode : 83; flag : umf_noinfo; reserved: 0),
       (unicode : 84; flag : umf_noinfo; reserved: 0),
       (unicode : 85; flag : umf_noinfo; reserved: 0),
       (unicode : 86; flag : umf_noinfo; reserved: 0),
       (unicode : 87; flag : umf_noinfo; reserved: 0),
       (unicode : 88; flag : umf_noinfo; reserved: 0),
       (unicode : 89; flag : umf_noinfo; reserved: 0),
       (unicode : 90; flag : umf_noinfo; reserved: 0),
       (unicode : 91; flag : umf_noinfo; reserved: 0),
       (unicode : 92; flag : umf_noinfo; reserved: 0),
       (unicode : 93; flag : umf_noinfo; reserved: 0),
       (unicode : 94; flag : umf_noinfo; reserved: 0),
       (unicode : 95; flag : umf_noinfo; reserved: 0),
       (unicode : 96; flag : umf_noinfo; reserved: 0),
       (unicode : 97; flag : umf_noinfo; reserved: 0),
       (unicode : 98; flag : umf_noinfo; reserved: 0),
       (unicode : 99; flag : umf_noinfo; reserved: 0),
       (unicode : 100; flag : umf_noinfo; reserved: 0),
       (unicode : 101; flag : umf_noinfo; reserved: 0),
       (unicode : 102; flag : umf_noinfo; reserved: 0),
       (unicode : 103; flag : umf_noinfo; reserved: 0),
       (unicode : 104; flag : umf_noinfo; reserved: 0),
       (unicode : 105; flag : umf_noinfo; reserved: 0),
       (unicode : 106; flag : umf_noinfo; reserved: 0),
       (unicode : 107; flag : umf_noinfo; reserved: 0),
       (unicode : 108; flag : umf_noinfo; reserved: 0),
       (unicode : 109; flag : umf_noinfo; reserved: 0),
       (unicode : 110; flag : umf_noinfo; reserved: 0),
       (unicode : 111; flag : umf_noinfo; reserved: 0),
       (unicode : 112; flag : umf_noinfo; reserved: 0),
       (unicode : 113; flag : umf_noinfo; reserved: 0),
       (unicode : 114; flag : umf_noinfo; reserved: 0),
       (unicode : 115; flag : umf_noinfo; reserved: 0),
       (unicode : 116; flag : umf_noinfo; reserved: 0),
       (unicode : 117; flag : umf_noinfo; reserved: 0),
       (unicode : 118; flag : umf_noinfo; reserved: 0),
       (unicode : 119; flag : umf_noinfo; reserved: 0),
       (unicode : 120; flag : umf_noinfo; reserved: 0),
       (unicode : 121; flag : umf_noinfo; reserved: 0),
       (unicode : 122; flag : umf_noinfo; reserved: 0),
       (unicode : 123; flag : umf_noinfo; reserved: 0),
       (unicode : 124; flag : umf_noinfo; reserved: 0),
       (unicode : 125; flag : umf_noinfo; reserved: 0),
       (unicode : 126; flag : umf_noinfo; reserved: 0),
       (unicode : 127; flag : umf_noinfo; reserved: 0),
       (unicode : 128; flag : umf_noinfo; reserved: 0),
       (unicode : 129; flag : umf_noinfo; reserved: 0),
       (unicode : 130; flag : umf_noinfo; reserved: 0),
       (unicode : 131; flag : umf_noinfo; reserved: 0),
       (unicode : 132; flag : umf_noinfo; reserved: 0),
       (unicode : 133; flag : umf_noinfo; reserved: 0),
       (unicode : 134; flag : umf_noinfo; reserved: 0),
       (unicode : 135; flag : umf_noinfo; reserved: 0),
       (unicode : 136; flag : umf_noinfo; reserved: 0),
       (unicode : 137; flag : umf_noinfo; reserved: 0),
       (unicode : 138; flag : umf_noinfo; reserved: 0),
       (unicode : 139; flag : umf_noinfo; reserved: 0),
       (unicode : 140; flag : umf_noinfo; reserved: 0),
       (unicode : 141; flag : umf_noinfo; reserved: 0),
       (unicode : 142; flag : umf_noinfo; reserved: 0),
       (unicode : 143; flag : umf_noinfo; reserved: 0),
       (unicode : 144; flag : umf_noinfo; reserved: 0),
       (unicode : 145; flag : umf_noinfo; reserved: 0),
       (unicode : 146; flag : umf_noinfo; reserved: 0),
       (unicode : 147; flag : umf_noinfo; reserved: 0),
       (unicode : 148; flag : umf_noinfo; reserved: 0),
       (unicode : 149; flag : umf_noinfo; reserved: 0),
       (unicode : 150; flag : umf_noinfo; reserved: 0),
       (unicode : 151; flag : umf_noinfo; reserved: 0),
       (unicode : 152; flag : umf_noinfo; reserved: 0),
       (unicode : 153; flag : umf_noinfo; reserved: 0),
       (unicode : 154; flag : umf_noinfo; reserved: 0),
       (unicode : 155; flag : umf_noinfo; reserved: 0),
       (unicode : 156; flag : umf_noinfo; reserved: 0),
       (unicode : 157; flag : umf_noinfo; reserved: 0),
       (unicode : 158; flag : umf_noinfo; reserved: 0),
       (unicode : 159; flag : umf_noinfo; reserved: 0),
       (unicode : 160; flag : umf_noinfo; reserved: 0),
       (unicode : 161; flag : umf_noinfo; reserved: 0),
       (unicode : 162; flag : umf_noinfo; reserved: 0),
       (unicode : 163; flag : umf_noinfo; reserved: 0),
       (unicode : 164; flag : umf_noinfo; reserved: 0),
       (unicode : 165; flag : umf_noinfo; reserved: 0),
       (unicode : 166; flag : umf_noinfo; reserved: 0),
       (unicode : 167; flag : umf_noinfo; reserved: 0),
       (unicode : 168; flag : umf_noinfo; reserved: 0),
       (unicode : 169; flag : umf_noinfo; reserved: 0),
       (unicode : 170; flag : umf_noinfo; reserved: 0),
       (unicode : 171; flag : umf_noinfo; reserved: 0),
       (unicode : 172; flag : umf_noinfo; reserved: 0),
       (unicode : 173; flag : umf_noinfo; reserved: 0),
       (unicode : 174; flag : umf_noinfo; reserved: 0),
       (unicode : 175; flag : umf_noinfo; reserved: 0),
       (unicode : 176; flag : umf_noinfo; reserved: 0),
       (unicode : 177; flag : umf_noinfo; reserved: 0),
       (unicode : 178; flag : umf_noinfo; reserved: 0),
       (unicode : 179; flag : umf_noinfo; reserved: 0),
       (unicode : 180; flag : umf_noinfo; reserved: 0),
       (unicode : 181; flag : umf_noinfo; reserved: 0),
       (unicode : 182; flag : umf_noinfo; reserved: 0),
       (unicode : 183; flag : umf_noinfo; reserved: 0),
       (unicode : 184; flag : umf_noinfo; reserved: 0),
       (unicode : 185; flag : umf_noinfo; reserved: 0),
       (unicode : 186; flag : umf_noinfo; reserved: 0),
       (unicode : 187; flag : umf_noinfo; reserved: 0),
       (unicode : 188; flag : umf_noinfo; reserved: 0),
       (unicode : 189; flag : umf_noinfo; reserved: 0),
       (unicode : 190; flag : umf_noinfo; reserved: 0),
       (unicode : 191; flag : umf_noinfo; reserved: 0),
       (unicode : 192; flag : umf_noinfo; reserved: 0),
       (unicode : 193; flag : umf_noinfo; reserved: 0),
       (unicode : 194; flag : umf_noinfo; reserved: 0),
       (unicode : 195; flag : umf_noinfo; reserved: 0),
       (unicode : 196; flag : umf_noinfo; reserved: 0),
       (unicode : 197; flag : umf_noinfo; reserved: 0),
       (unicode : 198; flag : umf_noinfo; reserved: 0),
       (unicode : 199; flag : umf_noinfo; reserved: 0),
       (unicode : 200; flag : umf_noinfo; reserved: 0),
       (unicode : 201; flag : umf_noinfo; reserved: 0),
       (unicode : 202; flag : umf_noinfo; reserved: 0),
       (unicode : 203; flag : umf_noinfo; reserved: 0),
       (unicode : 204; flag : umf_noinfo; reserved: 0),
       (unicode : 205; flag : umf_noinfo; reserved: 0),
       (unicode : 206; flag : umf_noinfo; reserved: 0),
       (unicode : 207; flag : umf_noinfo; reserved: 0),
       (unicode : 208; flag : umf_noinfo; reserved: 0),
       (unicode : 209; flag : umf_noinfo; reserved: 0),
       (unicode : 210; flag : umf_noinfo; reserved: 0),
       (unicode : 211; flag : umf_noinfo; reserved: 0),
       (unicode : 212; flag : umf_noinfo; reserved: 0),
       (unicode : 213; flag : umf_noinfo; reserved: 0),
       (unicode : 214; flag : umf_noinfo; reserved: 0),
       (unicode : 215; flag : umf_noinfo; reserved: 0),
       (unicode : 216; flag : umf_noinfo; reserved: 0),
       (unicode : 217; flag : umf_noinfo; reserved: 0),
       (unicode : 218; flag : umf_noinfo; reserved: 0),
       (unicode : 219; flag : umf_noinfo; reserved: 0),
       (unicode : 220; flag : umf_noinfo; reserved: 0),
       (unicode : 221; flag : umf_noinfo; reserved: 0),
       (unicode : 222; flag : umf_noinfo; reserved: 0),
       (unicode : 223; flag : umf_noinfo; reserved: 0),
       (unicode : 224; flag : umf_noinfo; reserved: 0),
       (unicode : 225; flag : umf_noinfo; reserved: 0),
       (unicode : 226; flag : umf_noinfo; reserved: 0),
       (unicode : 227; flag : umf_noinfo; reserved: 0),
       (unicode : 228; flag : umf_noinfo; reserved: 0),
       (unicode : 229; flag : umf_noinfo; reserved: 0),
       (unicode : 230; flag : umf_noinfo; reserved: 0),
       (unicode : 231; flag : umf_noinfo; reserved: 0),
       (unicode : 232; flag : umf_noinfo; reserved: 0),
       (unicode : 233; flag : umf_noinfo; reserved: 0),
       (unicode : 234; flag : umf_noinfo; reserved: 0),
       (unicode : 235; flag : umf_noinfo; reserved: 0),
       (unicode : 236; flag : umf_noinfo; reserved: 0),
       (unicode : 237; flag : umf_noinfo; reserved: 0),
       (unicode : 238; flag : umf_noinfo; reserved: 0),
       (unicode : 239; flag : umf_noinfo; reserved: 0),
       (unicode : 240; flag : umf_noinfo; reserved: 0),
       (unicode : 241; flag : umf_noinfo; reserved: 0),
       (unicode : 242; flag : umf_noinfo; reserved: 0),
       (unicode : 243; flag : umf_noinfo; reserved: 0),
       (unicode : 244; flag : umf_noinfo; reserved: 0),
       (unicode : 245; flag : umf_noinfo; reserved: 0),
       (unicode : 246; flag : umf_noinfo; reserved: 0),
       (unicode : 247; flag : umf_noinfo; reserved: 0),
       (unicode : 248; flag : umf_noinfo; reserved: 0),
       (unicode : 249; flag : umf_noinfo; reserved: 0),
       (unicode : 250; flag : umf_noinfo; reserved: 0),
       (unicode : 251; flag : umf_noinfo; reserved: 0),
       (unicode : 252; flag : umf_noinfo; reserved: 0),
       (unicode : 253; flag : umf_noinfo; reserved: 0),
       (unicode : 254; flag : umf_noinfo; reserved: 0),
       (unicode : 255; flag : umf_noinfo; reserved: 0)
     );

     unicodemap : tunicodemap = (
       cpname : '8859-1';
       cp : 28591;
       map : @map;
       lastchar : 255;
       next : nil;
       internalmap : true
     );

  var
     hp : punicodemap;

initialization
  mappings:=nil;
  registermapping(@unicodemap)
finalization
  while assigned(mappings) do
    begin
       hp:=mappings^.next;
       if not(mappings^.internalmap) then
         begin
            freemem(mappings^.map);
            dispose(mappings);
         end;
       mappings:=hp;
    end;
end.
