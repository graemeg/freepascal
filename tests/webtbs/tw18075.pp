{$codepage UTf8}
Var cad1:unicodeString;
    cad2:Widestring;
    n:integer;
Begin
  cad1:='犮犯狃狄狪独';
  cad2:=cad1;

  //Unicodestring, 1 character is ok
  Writeln('unicodestring');
  n:=pos('犮',cad1);
  Writeln(n);
  if n<>1 then
    halt(1);

  Writeln('widestring');
  n:=pos('犮',cad2);
  Writeln(n);
  if n<>1 then
    halt(1);

  //Unicodestring, more charactere wrong
  Writeln('unicodestring');
  n:=pos('狃狄',cad1);
  Writeln(n);  //show position 0
  if n<>3 then
    halt(1);

  Writeln('widestring');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  n:=pos(WideString('狃狄'),cad2); //Is correct position 3
=======
  n:=pos('狃狄',cad2); //Is correct position 3
>>>>>>> graemeg/cpstrnew
=======
  n:=pos('狃狄',cad2); //Is correct position 3
>>>>>>> graemeg/cpstrnew
=======
  n:=pos('狃狄',cad2); //Is correct position 3
>>>>>>> graemeg/cpstrnew
=======
  n:=pos('狃狄',cad2); //Is correct position 3
>>>>>>> origin/cpstrnew

  Writeln(n);
  if n<>3 then
    halt(1);

  Writeln('ok');
End.
