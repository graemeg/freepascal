program unblockchm;

// Marco van de Voort
// BSD license 
// Quick and dirty scritp to unblocks CHMs on xpsp2/vista/w7
//
// todo : populatefiles needs fix for when filespec contains a directory.
//
// based on http://stackoverflow.com/questions/1617509/unblock-a-file-with-powershell

{$mode delphi}
uses sysutils,classes;

procedure usage;

begin
  writeln('unblockchm. Unblocks chms in XPsp2,vista,w7  (C) 2010 Marco van de Voort');
  writeln;
  Writeln('usage: unblockchm <filespec> <filespec2> ..');
  writeln;
  writeln('<filespec> may contain basic wildcards.');
  writeln;
end;

procedure unblockchm(s:string);
var f : file;
begin
 writeln('unblocking ',s);
 assignfile(f,s+':Zone.Identifier');
 rewrite(f,1);
 truncate(f);
 closefile(f);
end;

procedure populatefiles(files:TStringlist;filespec:string);
var
  searchResult : TSearchRec;
begin
 if FindFirst(filespec, faAnyFile, searchResult) = 0 then
  begin
    repeat
      files.add(searchresult.name);
    until FindNext(searchResult) <> 0;
    // Must free up resources used by these successful finds
    FindClose(searchResult);
  end;
end;

var files : TStringList;
    i : Integer;

begin
 if paramcount=0 then
   begin
     Usage;
     halt;
   end;
 files :=TStringList.create;
 for i:=1 to paramcount do
  populatefiles(files,paramstr(i));
 if files.count>0 then
   for i:=0 to files.count-1 do
     unblockchm(files[i]);
end.

