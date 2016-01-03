{ %skiptarget=go32v2 }

program MaxThread;

{$mode objfpc}{$H+}

uses
 {$ifdef unix}
 cthreads,
 {$endif}
 Classes, SysUtils;
 { add your units here }


type
 ThProva = class(TThread)
 private
   Number: Integer;
 protected
   procedure Execute; override;
 public
   constructor Create(ThreadNumber: Integer);
 end;

var
 threadsfinished: Integer;


constructor ThProva.Create(ThreadNumber: Integer);
begin
 self.Number:= ThreadNumber;
 self.FreeOnTerminate:= true;
 inherited Create(true);
end;

procedure ThProva.Execute;
begin
 WriteLn('I am the thread number '+ IntToStr(Number));
 Terminate;
 { not exactly finished yet, but good enough }
 interlockedincrement(threadsfinished);
end;


var
 ThreadNumber: Integer;
 thr: thprova;
begin
 ThreadNumber:= 0;
 WriteLn('Begin');
 { needs to be > 410 because at least on Mac OS X and Linux you can start }
 { about 400 threads before you run into trouble if they aren't finished  }
 while (threadnumber < 500) do
   begin
     try
       Inc(ThreadNumber);
       thr:=ThProva.Create(Threadnumber);
<<<<<<< HEAD
<<<<<<< HEAD
       thr.start;
     Except on e: Exception do
       begin
         { maybe the previously started threads didn't finish yet -> give them some
           more time to completely terminate }
         sleep(1000);
         try
           thr:=ThProva.Create(Threadnumber);
           thr.start;
         except
           WriteLn(e.Message);
           halt(1);
         end;
       end;
     end;
=======
=======
>>>>>>> origin/fixes_2_2
       thr.resume;
     Except on e: Exception do
       begin
         WriteLn(e.Message);
         halt(1);
       end;
     end;
     if (threadnumber mod 16) = 0 then
       while (threadsfinished<>threadnumber) do
         { let the started threads to finish }
         sleep(70);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   end;
  while (threadsfinished<>threadnumber) do
    { give some time to the started threads to finish }
    sleep(100);
  sleep(200)
end.
