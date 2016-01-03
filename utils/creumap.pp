{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000 by Florian Klaempfl

    It creates pascal units from unicode mapping files

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program creumap;

  uses
     charset;

  procedure doerror;

    begin
       writeln('Usage: creumap <cpname> <cpnumber>');
       writeln('cpname : A mapping file called <cpname>.txt must be present');
       writeln('cpnumber : the code page number');
       halt(1);
    end;

type
  TEndianKind = (Little, Big);
const
  ENDIAN_SUFFIX : array[TEndianKind] of string[2] = ('le','be');
{$IFDEF ENDIAN_LITTLE}
  ENDIAN_NATIVE     = TEndianKind.Little;
  ENDIAN_NON_NATIVE = TEndianKind.Big;
{$ENDIF ENDIAN_LITTLE}
{$IFDEF ENDIAN_BIG}
  ENDIAN_NATIVE = TEndianKind.Big;
  ENDIAN_NON_NATIVE = TEndianKind.Little;
{$ENDIF ENDIAN_BIG}

  procedure CreateBinaryFile(AMap : punicodemap; const ABaseFile : string);

    var
       nef, oef : File of Byte;
       h, th : TSerializedMapHeader;
       k : Longint;
       um : tunicodecharmapping;
       pum : punicodecharmapping;
       rm : treversecharmapping;
       prm : preversecharmapping;
    begin
       FillChar(h,SizeOf(h),0);
       h.cpName := AMap^.cpname;
       h.cp := AMap^.cp;
       h.lastChar := AMap^.lastchar;
       h.mapLength := (AMap^.lastchar+1)*SizeOf(tunicodecharmapping);
       h.reverseMapLength := AMap^.reversemaplength*SizeOf(treversecharmapping);
       Assign(nef,(ABaseFile+'_'+ENDIAN_SUFFIX[ENDIAN_NATIVE]+'.bcm'));
       Rewrite(nef);
       BlockWrite(nef,h,SizeOf(h));
       BlockWrite(nef,AMap^.map^,h.mapLength);
       BlockWrite(nef,AMap^.reversemap^,h.reverseMapLength);
       Close(nef);

       th.cpName := h.cpName;
       th.cp := SwapEndian(h.cp);
       th.mapLength := SwapEndian(h.mapLength);
       th.lastChar := SwapEndian(h.lastChar);
       th.reverseMapLength := SwapEndian(h.reverseMapLength);
       Assign(oef,(ABaseFile+'_'+ENDIAN_SUFFIX[ENDIAN_NON_NATIVE]+'.bcm'));
       Rewrite(oef);
       BlockWrite(oef,th,SizeOf(th));
       pum := AMap^.map;
       for k := 0 to AMap^.lastchar do begin
          um.flag := pum^.flag;
          um.reserved := pum^.reserved;
          um.unicode := SwapEndian(pum^.unicode);
          BlockWrite(oef,um,SizeOf(um));
          Inc(pum);
       end;
       prm := AMap^.reversemap;
       for k := 0 to AMap^.reversemaplength - 1 do begin
         rm.unicode := SwapEndian(prm^.unicode);
         rm.char1 := prm^.char1;
         rm.char2 := prm^.char2;
         BlockWrite(oef,rm,SizeOf(rm));
         Inc(prm);
       end;
       Close(oef);
    end;

  var
     p : punicodemap;
     i : longint;
     t : text;
     e : word;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     c : longint;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

begin
   if paramcount<>2 then
     doerror;
   Val(paramstr(2),i,e);
   if e<>0 then
     doerror;
     
   p:=loadunicodemapping(paramstr(1),paramstr(1)+'.txt',i);
   if p=nil then
     doerror;
   assign(t,paramstr(1)+'.pp');
   rewrite(t);
   writeln(t,'{ This is an automatically created file, so don''t edit it }');
   writeln(t,'unit ',p^.cpname,';');
   writeln(t);
   writeln(t,'  interface');
   writeln(t);
   writeln(t,'  implementation');
   writeln(t);
   writeln(t,'  uses');
   writeln(t,'     charset;');
   writeln(t);
   writeln(t,'  const');
   writeln(t,'     map : array[0..',p^.lastchar,'] of tunicodecharmapping = (');
   for i:=0 to p^.lastchar do
     begin
        write(t,'       (unicode : ',p^.map[i].unicode,'; flag : ');
        case p^.map[i].flag of
           umf_noinfo : write(t,'umf_noinfo');
           umf_leadbyte : write(t,'umf_leadbyte');
           umf_undefined : write(t,'umf_undefined');
           umf_unused : write(t,'umf_unused');
        end;
        write(t,'; reserved: 0)');
        if i<>p^.lastchar then
          writeln(t,',')
        else
          writeln(t);
     end;
   writeln(t,'     );');
   writeln(t);
   c:=p^.reversemaplength-1;
   writeln(t,'     reversemap : array[0..',c,'] of treversecharmapping = (');
   for i:=0 to c do
     begin
        write(t,'       (',
                'unicode : ',p^.reversemap[i].unicode,
                '; char1 : ',p^.reversemap[i].char1,
                '; char2 : ',p^.reversemap[i].char2,
                ')'
        );
        if i<>c then
          writeln(t,',')
        else
          writeln(t);
     end;
   writeln(t,'     );');
   writeln(t);
   writeln(t,'     unicodemap : tunicodemap = (');
   writeln(t,'       cpname : ''',p^.cpname,''';');
   writeln(t,'       cp : ',p^.cp,';');
   writeln(t,'       map : @map;');
   writeln(t,'       lastchar : ',p^.lastchar,';');
   writeln(t,'       reversemap : @reversemap;');
   writeln(t,'       reversemaplength : ',p^.reversemaplength,';');
   writeln(t,'       next : nil;');
   writeln(t,'       internalmap : true');
   writeln(t,'     );');
   writeln(t);
   writeln(t,'  begin');
   writeln(t,'     registermapping(@unicodemap)');
   writeln(t,'  end.');
   close(t);

   CreateBinaryFile(p,paramstr(1));
end.
