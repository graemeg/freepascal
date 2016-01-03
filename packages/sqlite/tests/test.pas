program test;

uses sqlite,sqlitedb, strings,classes;

var
  MySQL: TSQLite;
  SQL: String;
  i, j: Integer;
  a: TStringList;
begin
  Writeln('Creating class');
  MySQL := TSQLite.Create('test.db');
  MySQL.BusyTimeout := 1000;

 // writeln(MySQL.Version);
  Writeln('Creating table');
  SQL := 'CREATE TABLE Test(No int, Nom varchar(32),Prenom varchar(32));';
  MySQL.Query(sql, nil);
  SQL := 'INSERT INTO Test VALUES(1,''Coursiere'', ''Olivier'');';
  if MySQL.IsComplete(sql) then
    begin
    Writeln('Inserting first row');
    MySQL.Query(sql, nil);
    end;
  SQL := 'INSERT INTO Test VALUES(2,''Jourde'', ''Eric'');';
  if MySQL.IsComplete(sql) then
    begin
    Writeln('Inserting second row') ;
    MySQL.Query(sql, nil);
    end;
  Writeln('Selecting rows') ;

  SQL := 'SELECT * FROM Test;';
  MySQL.Query(sql, nil);
  writeln('Fields Names -------------------');
  for i:=0 to MySQL.List_FieldName.count-1 do
    writeln(i,' -> ',MySQL.List_FieldName.Strings[i]);
  writeln('Fields -------------------');
  for i:=0 to MySQL.List_Field.count-1 do
      begin
        a:=TStringList(MySQL.List_Field.items[i]);
        write(i,' -> ');
        for j:=0 to a.count-1 do
          write(a.Strings[j],'  ');
        writeln('');
      end;

// Uncomment to remove table again.
//  SQL := 'DROP TABLE Test;';
//  MySQL.Query(sql, nil);
  MySQL.Free;
end.
