uses
  Sysutils;
var
  f : file;  
Begin
  Writeln('IsFileNameCaseSensitive: ',IsFileNameCaseSensitive('tfilenamecase1.dat'));
  Writeln('IsFileNameCasePreserving ',IsFileNameCasePreserving('tfilenamecase1.dat'));
  Assign(f,'tfilenamecase1.dat');
  Rewrite(f);
  Close(f);
  if IsFileNameCaseSensitive('tfilenamecase1.dat') then
    begin
      if FileExists('Tfilenamecase1.dat') then
        halt(1);
    end
  else
    begin
      if not(FileExists('Tfilenamecase1.dat')) then
        halt(2);
    end;
  if IsFileNameCasePreserving('tfilenamecase1.dat') then
    begin
      if IsFileNameCaseSensitive('tfilenamecase1.dat') then
        begin
          if FileExists('Tfilenamecase1.dat') then
            halt(1);
        end
      else
        begin
          if not(FileExists('Tfilenamecase1.dat')) then
            halt(1);
        end
    end;
End.

  
  