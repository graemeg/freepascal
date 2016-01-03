program storedfalse;
{$ifdef FPC}{$mode objfpc}{$h+}{$INTERFACES CORBA}{$endif}
{$ifdef mswindows}{$apptype console}{$endif}
uses
<<<<<<< HEAD
<<<<<<< HEAD
 {$ifdef FPC}{$ifdef unix}cthreads,{$endif}{$endif}
=======
 {$ifdef FPC}{$ifdef linux}cthreads,{$endif}{$endif}
>>>>>>> graemeg/fixes_2_2
=======
 {$ifdef FPC}{$ifdef linux}cthreads,{$endif}{$endif}
>>>>>>> origin/fixes_2_2
 sysutils,classes;

type
 tenum = (eena,eenb,eenc,eend,eene,eenf,eeng,eenh,eeni);
 tset = set of tenum;

 ttestclass1 = class(tcomponent)
  private
   fprop1: tset;
  public
   property prop1: tset read fprop1 write fprop1 stored true;
 end;

 ttestclass2 = class(ttestclass1)
  published
   property prop1;
 end;

var
 testclass2,testclass3: ttestclass2;
 stream1,stream2: tmemorystream;
 str1: ansistring;
begin
 testclass2:= ttestclass2.create(nil);
 testclass2.prop1:= [eenb,eend,eene,eenh,eeni];
 stream1:= tmemorystream.create;
 try
  stream1.writecomponent(testclass2);
  stream2:= tmemorystream.create;
  try
   stream1.position:= 0;
   objectbinarytotext(stream1,stream2);
   stream1.position:= 0;
   stream2.position:= 0;
   setlength(str1,stream2.size);
   move(stream2.memory^,str1[1],length(str1));
   writeln(str1);
   testclass3:=ttestclass2.create(nil);
   stream1.readcomponent(testclass3);
   if (testclass3.prop1<>[eenb,eend,eene,eenh,eeni]) then
     halt(1);
  finally
   stream2.free;
  end;
 finally
  stream1.free;
 end;
end.
