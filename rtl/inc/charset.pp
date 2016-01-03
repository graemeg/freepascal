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
{$pointermath on}
{$PACKENUM 1}
unit charset;

  interface

    type
       tunicodechar = word;
       tunicodestring = ^tunicodechar;

       tunicodecharmappingflag = (umf_noinfo,umf_leadbyte,umf_undefined,
         umf_unused);

       punicodecharmapping = ^tunicodecharmapping;
       tunicodecharmapping = packed record
          unicode : tunicodechar;
          flag : tunicodecharmappingflag;
          reserved : byte;
       end;

       preversecharmapping = ^treversecharmapping;
       treversecharmapping = packed record
          unicode : tunicodechar;
          char1   : Byte;
          char2   : Byte;
       end;

       punicodemap = ^tunicodemap;
       tunicodemap = record
          cpname : string[20];
          cp : word;
          map : punicodecharmapping;
          lastchar : longint;
          reversemap : preversecharmapping;
          reversemaplength : longint;
          next : punicodemap;
          internalmap : boolean;
       end;

       TSerializedMapHeader = packed record
          cpName           : string[20];
          cp               : UInt16;
          mapLength        : UInt32;
          lastChar         : Int32;
          reverseMapLength : UInt32;
       end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    const
      BINARY_MAPPING_FILE_EXT = '.bcm';

    function loadunicodemapping(const cpname,f : string; cp :word) : punicodemap;
    function loadbinaryunicodemapping(const directory,cpname : string) : punicodemap;overload;
    function loadbinaryunicodemapping(const filename : string) : punicodemap;overload;
    function loadbinaryunicodemapping(
      const AData       : Pointer;
      const ADataLength : Integer
    ) : punicodemap;overload;
    procedure registermapping(p : punicodemap);
    function registerbinarymapping(const directory,cpname : string):Boolean;
    function getmap(const s : string) : punicodemap;
    function getmap(cp : word) : punicodemap;
    function mappingavailable(const s : string) : boolean;inline;
    function mappingavailable(cp :word) : boolean;inline;
    function getunicode(c : char;p : punicodemap) : tunicodechar;inline;
    function getunicode(
      AAnsiStr : pansichar;
      AAnsiLen : LongInt;
      AMap     : punicodemap;
      ADest    : tunicodestring
    ) : LongInt;
=======
    function loadunicodemapping(const cpname,f : string; cp :word) : punicodemap;
    procedure registermapping(p : punicodemap);
=======
    function loadunicodemapping(const cpname,f : string; cp :word) : punicodemap;
    procedure registermapping(p : punicodemap);
>>>>>>> graemeg/cpstrnew
=======
    function loadunicodemapping(const cpname,f : string; cp :word) : punicodemap;
    procedure registermapping(p : punicodemap);
>>>>>>> graemeg/cpstrnew
=======
    function loadunicodemapping(const cpname,f : string; cp :word) : punicodemap;
    procedure registermapping(p : punicodemap);
>>>>>>> origin/cpstrnew
    function getmap(const s : string) : punicodemap; 
    function getmap(cp : word) : punicodemap;   
    function mappingavailable(const s : string) : boolean;
    function mappingavailable(cp :word) : boolean;
    function getunicode(c : char;p : punicodemap) : tunicodechar;
>>>>>>> graemeg/cpstrnew
    function getascii(c : tunicodechar;p : punicodemap) : string;
    function getascii(c : tunicodechar;p : punicodemap; ABuffer : PAnsiChar; ABufferLen : LongInt) : LongInt;

  implementation

    const
      UNKNOW_CHAR_A = ansichar(63);
      UNKNOW_CHAR_W = tunicodechar(63);
    var
       mappings : punicodemap;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    procedure QuickSort(AList: preversecharmapping; L, R : Longint);
    var
      I, J : Longint;
      P, Q : treversecharmapping;
    begin
      repeat
        I:=L;
        J:=R;
        P:=AList[(L + R) div 2];
        repeat
          while (P.unicode-AList[I].unicode) > 0 do
            I:=I+1;
          while (P.unicode-AList[J].unicode) < 0 do
            J:=J-1;
          if I<=J then
            begin
              Q:=AList[I];
              AList[I]:=AList[J];
              AList[J]:=Q;
              I:=I+1;
              J:=J-1;
            end;
        until I > J;
        if J-L < R-I then
          begin
            if L<J then
              QuickSort(AList, L, J);
            L:=I;
          end
        else
          begin
            if I < R then
              QuickSort(AList, I, R);
            R:=J;
          end;
      until L>=R;
    end;

    function find(
      const c     : tunicodechar;
      const AData : preversecharmapping;
      const ALen  : LongInt
    ) : preversecharmapping;overload;
    var
       l, h, m : longint;
       r:preversecharmapping;
    begin
      if ALen=0 then
        exit(nil);
      r:=AData;
      l:=0;
      h:=ALen-1;
      while l<h do begin
        m:=(l+h) div 2;
        if r[m].unicode<c then
          l:=m+1
        else
          h:=m;
      end;
      if (l=h) and (r[l].unicode=c) then
        Result:=@r[l]
      else
        Result:=nil;
    end;

    function find(
      const c : tunicodechar;
      const p : punicodemap
    ) : preversecharmapping;overload;inline;
    begin
      Result:=find(c,p^.reversemap,p^.reversemaplength);
    end;

    function RemoveDuplicates(
      const AData      : preversecharmapping;
      const ALen       : LongInt;
      out   AResultLen : LongInt
    ) : preversecharmapping;
    var
      r0, r, p, t : preversecharmapping;
      i, c, actualCount : LongInt;
    begin
      c:=ALen;
      GetMem(r0,c*SizeOf(treversecharmapping));
      r:=r0;
      p:=AData;
      actualCount:=0;
      i:=0;
      while i<c do
        begin
          t:=find(p^.unicode,r0,actualCount);
          if t=nil then
            begin
              r^:=p^;
              actualCount:=actualCount+1;
              Inc(r);
            end
          else
            begin
              if (p^.char1<t^.char1) or
                 ((p^.char1=t^.char1) and (p^.char2<t^.char2))
              then
                t^:=p^;//keep the first mapping
            end;
          i:=i+1;
          Inc(p);
        end;
      if c<>actualCount then
        ReAllocMem(r0,actualCount*SizeOf(treversecharmapping));
      AResultLen:=actualCount;
      Result:=r0;
    end;

    function buildreversemap(
      const AMapping   : punicodecharmapping;
      const ALen       : LongInt;
      out   AResultLen : LongInt
    ) : preversecharmapping;
    var
      r0, r, t : preversecharmapping;
      i, c, actualCount, ti : LongInt;
      p : punicodecharmapping;
    begin
      if (ALen<1) then
        exit(nil);
      p:=AMapping;
      c:=ALen;
      GetMem(r0,c*SizeOf(treversecharmapping));
      r:=r0;
      actualCount:=0;
      i:=0;
      while i<c do
        begin
          if (p^.flag=umf_noinfo) then
            begin
              r^.unicode:=p^.unicode;
              if i<=High(Byte) then
                begin
                  r^.char1:=i;
                  r^.char2:=0;
                end
              else
                begin
                  r^.char1:=i div 256;
                  r^.char2:=i mod 256;
                end;
              actualCount:=actualCount+1;
              Inc(r);
            end;
          Inc(p);
          i:=i+1;
        end;
      if c<>actualCount then
        ReAllocMem(r0,actualCount*SizeOf(treversecharmapping));
      if actualCount>1 then
        begin
          QuickSort(r0,0,(actualCount-1));
          t:=RemoveDuplicates(r0,actualCount,ti);
          FreeMem(r0,actualCount*SizeOf(treversecharmapping));
          r0:=t;
          actualCount:=ti;
        end;
      AResultLen:=actualCount;
      Result:=r0;
    end;

    procedure inititems(const p : punicodecharmapping; const ALen : LongInt);
    const
      INIT_ITEM : tunicodecharmapping = (unicode:0; flag:umf_unused; reserved:0);
    var
      x : punicodecharmapping;
      i : LongInt;
    begin
      x:=p;
      for i:=0 to ALen-1 do
        begin
          x^:=INIT_ITEM;
          Inc(x);
        end;
    end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
         lastchar, i : longint;

      begin
         lastchar:=-1;
         loadunicodemapping:=nil;
         datasize:=256;
         GetMem(data,sizeof(tunicodecharmapping)*datasize);
         inititems(data,datasize);
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
                             i:=datasize;
                             datasize:=charpos+8*1024;
                             reallocmem(data,sizeof(tunicodecharmapping)*datasize);
                             inititems(@data[i],(datasize-i));
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
         p^.reversemap:=buildreversemap(p^.map,(p^.lastchar+1),p^.reversemaplength);
         loadunicodemapping:=p;
      end;


    function loadbinaryunicodemapping(const directory, cpname : string) : punicodemap;
      const
        {$IFDEF ENDIAN_LITTLE}
          ENDIAN_SUFFIX = 'le';
        {$ENDIF ENDIAN_LITTLE}
        {$IFDEF ENDIAN_BIG}
          ENDIAN_SUFFIX = 'be';
        {$ENDIF ENDIAN_BIG}
      var
        fileName : string;
      begin
        fileName := directory;
        if (fileName <> '') then begin
          if (fileName[Length(fileName)] <> DirectorySeparator) then
            fileName := fileName + DirectorySeparator;
        end;
        fileName := fileName + cpname + '_' + ENDIAN_SUFFIX + BINARY_MAPPING_FILE_EXT;
        Result := loadbinaryunicodemapping(fileName);
      end;

    {$PUSH}
      {$I-}
    function loadbinaryunicodemapping(const filename : string) : punicodemap;
      const
        BLOCK_SIZE = 16*1024;
      var
        f : File of Byte;
        locSize, locReaded, c : LongInt;
        locBuffer : PByte;
        locBlockSize : LongInt;
      begin
        Result := nil;
        if (filename='') then
          exit;
        Assign(f,filename);
        Reset(f);
        if (IOResult<>0) then
          exit;
        locSize:=FileSize(f);
        if (locSize<SizeOf(TSerializedMapHeader)) then
          begin
            Close(f);
            exit;
          end;
        locBuffer:=GetMem(locSize);
        locBlockSize:=BLOCK_SIZE;
        locReaded:=0;
        c := 0;
        while (locReaded<locSize) do
          begin
            if (locBlockSize>(locSize-locReaded)) then
              locBlockSize:=locSize-locReaded;
            BlockRead(f,locBuffer[locReaded],locBlockSize,c);
            if (IOResult<>0) or (c<=0) then
              begin
                FreeMem(locBuffer,locSize);
                Close(f);
                exit;
              end;
            locReaded:=locReaded+c;
          end;
        Result:=loadbinaryunicodemapping(locBuffer,locSize);
        FreeMem(locBuffer,locSize);
        Close(f);
      end;
    {$POP}

    procedure freemapping(amapping : punicodemap);
      begin
        if (amapping = nil) then
          exit;
        if (amapping^.map <> nil) then
          freemem(amapping^.map);
        if (amapping^.reversemap <> nil) then
          freemem(amapping^.reversemap);
        dispose(amapping);
      end;

    function loadbinaryunicodemapping(
      const AData       : Pointer;
      const ADataLength : Integer
    ) : punicodemap;
      var
        dataPointer : PByte;
        readedLength : LongInt;

        function ReadBuffer(ADest : Pointer; ALength : LongInt) : Boolean;
          begin
            Result := (readedLength + ALength) <= ADataLength;
            if not result then
              exit;
            Move(dataPointer^,ADest^,ALength);
            Inc(dataPointer,ALength);
            readedLength := readedLength + ALength;
          end;

      var
        h : TSerializedMapHeader;
        r : punicodemap;
      begin
        Result := nil;
        readedLength := 0;
        dataPointer := AData;
        if not ReadBuffer(@h,SizeOf(h)) then
          exit;
        New(r);
        FillChar(r^,SizeOf(tunicodemap),0);
        r^.cpname := h.cpName;
        r^.cp := h.cp;
        r^.map := AllocMem(h.mapLength);
        if not ReadBuffer(r^.map,h.mapLength) then
          begin
            freemapping(r);
            exit;
          end;
        r^.lastchar := h.lastChar;
        r^.reversemap := AllocMem(h.reverseMapLength);
        if not ReadBuffer(r^.reversemap,h.reverseMapLength) then
          begin
            freemapping(r);
            exit;
          end;
        r^.reversemaplength := (h.reverseMapLength div SizeOf(treversecharmapping));
        Result := r;
      end;

    procedure registermapping(p : punicodemap);

      begin
         p^.next:=mappings;
         mappings:=p;
      end;

    {$ifdef FPC_HAS_FEATURE_THREADING}
    threadvar
    {$else FPC_HAS_FEATURE_THREADING}
    var
    {$endif FPC_HAS_FEATURE_THREADING}
      strmapcache : string;
      strmapcachep : punicodemap;

    function registerbinarymapping(const directory, cpname : string) : Boolean;
      var
        p : punicodemap;
      begin
        Result := False;
        p := loadbinaryunicodemapping(directory,cpname);
        if (p = nil) then
          exit;
        registermapping(p);
        Result := True;
      end;

    function getmap(const s : string) : punicodemap;

      var
         hp : punicodemap;

      begin
         if (strmapcache=s) and assigned(strmapcachep) and (strmapcachep^.cpname=s) then
           begin
              getmap:=strmapcachep;
              exit;
           end;
         hp:=mappings;
         while assigned(hp) do
           begin
              if hp^.cpname=s then
                begin
                   getmap:=hp;
                   strmapcache:=s;
                   strmapcachep:=hp;
                   exit;
                end;
              hp:=hp^.next;
           end;
         getmap:=nil;
      end;////////


    {$ifdef FPC_HAS_FEATURE_THREADING}
    threadvar
    {$else FPC_HAS_FEATURE_THREADING}
    var
    {$endif FPC_HAS_FEATURE_THREADING}
      intmapcache : word;
      intmapcachep : punicodemap;
    function getmap(cp : word) : punicodemap;

      var
         hp : punicodemap;

      begin
         if (intmapcache=cp) and assigned(intmapcachep) and (intmapcachep^.cp=cp) then
           begin
              getmap:=intmapcachep;
              exit;
           end;
         hp:=mappings;
         while assigned(hp) do
           begin
              if hp^.cp=cp then
                begin
                   getmap:=hp;
                   intmapcache:=cp;
                   intmapcachep:=hp;
                   exit;
                end;
              hp:=hp^.next;
           end;
         getmap:=nil;
      end;////////

    function getmap(cp : word) : punicodemap;

      var
         hp : punicodemap;

      const
         mapcache : word = 0;
         mapcachep : punicodemap = nil;

      begin
<<<<<<< HEAD
<<<<<<< HEAD
         if (mapcache=cp) and assigned(mapcachep) and (mapcachep^.cp=cp) then
=======
         if (mapcache=s) and assigned(mapcachep) and (mapcachep^.cpname=s) then
>>>>>>> graemeg/fixes_2_2
=======
         if (mapcache=s) and assigned(mapcachep) and (mapcachep^.cpname=s) then
>>>>>>> origin/fixes_2_2
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
      end;////////

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
      end;////////

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
      end;////////

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

    function getunicode(
      AAnsiStr : pansichar;
      AAnsiLen : LongInt;
      AMap     : punicodemap;
      ADest    : tunicodestring
    ) : LongInt;

      var
         i, c, k, destLen : longint;
         ps : pansichar;
         pd : ^tunicodechar;

      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if (AAnsiStr=nil) or (AAnsiLen<=0) then
          exit(0);
        ps:=AAnsiStr;
        if (ADest=nil) then
          begin
            c:=AAnsiLen-1;
            destLen:=0;
            i:=0;
            while (i<=c) do
              begin
                if (ord(ps^)<=AMap^.lastchar) then
                  begin
                    if (AMap^.map[ord(ps^)].flag=umf_leadbyte) and (i<c) then
                      begin
                        Inc(ps);
                        i:=i+1;
                      end;
                  end;
                i:=i+1;
                Inc(ps);
                destLen:=destLen+1;
              end;
            exit(destLen);
          end;

        pd:=ADest;
        c:=AAnsiLen-1;
        i:=0;
        while (i<=c) do
          begin
            if (ord(ps^)<=AMap^.lastchar) then
              begin
                if (AMap^.map[ord(ps^)].flag=umf_leadbyte) then
                  begin
                    if (i<c) then
                      begin
                        k:=(Ord(ps^)*256);
                        Inc(ps);
                        i:=i+1;
                        k:=k+Ord(ps^);
                        if (k<=AMap^.lastchar) then
                          pd^:=AMap^.map[k].unicode
                        else
                          pd^:=UNKNOW_CHAR_W;
                      end
                    else
                      pd^:=UNKNOW_CHAR_W;
                  end
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         { at least map to '?' }
         getascii:=#63;
         for i:=0 to p^.lastchar do
           if p^.map[i].unicode=c then
             begin
                if i<256 then
                  getascii:=chr(i)
>>>>>>> graemeg/cpstrnew
                else
                  pd^:=AMap^.map[ord(ps^)].unicode
              end
            else
              pd^:=UNKNOW_CHAR_W;
            i:=i+1;
            Inc(ps);
            Inc(pd);
          end;
        result:=((PtrUInt(pd)-PtrUInt(ADest)) div SizeOf(tunicodechar));
      end;

    function getascii(c : tunicodechar;p : punicodemap) : string;
      var
         rm : preversecharmapping;
      begin
        rm:=find(c,p);
        if rm<>nil then
          begin
            if rm^.char2=0 then
              begin
                SetLength(Result,1);
                Byte(Result[1]):=rm^.char1;
              end
            else
              begin
                SetLength(Result,2);
                Byte(Result[1]):=rm^.char1;
                Byte(Result[2]):=rm^.char2;
              end;
          end
        else
          Result:=UNKNOW_CHAR_A;
      end;

    function getascii(c : tunicodechar;p : punicodemap; ABuffer : PAnsiChar; ABufferLen : LongInt) : LongInt;
      var
         rm : preversecharmapping;
      begin
         if (ABuffer<>nil) and (ABufferLen<=0) then
           exit(-1);
        rm:=find(c,p);
        if rm<>nil then
          begin
            if (ABuffer=nil) then
              begin
                if rm^.char2=0 then
                  Result:=1
                else
                  Result:=2;
              end
            else
              begin
                if rm^.char2=0 then
                  begin
                    Byte(ABuffer^):=rm^.char1;
                    Result:=1;
                  end
                else
                  begin
                    if (ABufferLen<2) then
                      Result:=-1
                    else
                      begin
                        Byte(ABuffer^):=rm^.char1;
                        Byte((ABuffer+1)^):=rm^.char2;
                        Result:=2;
                      end
                  end;
              end;
          end
        else
          begin
            ABuffer^:=UNKNOW_CHAR_A;
            Result:=1;
          end;
      end;

  var
     hp : punicodemap;

initialization
  mappings:=nil;
finalization
  while assigned(mappings) do
    begin
       hp:=mappings^.next;
       if not(mappings^.internalmap) then
         begin
            freemem(mappings^.map);
            freemem(mappings^.reversemap);
            dispose(mappings);
         end;
       mappings:=hp;
    end;
end.
