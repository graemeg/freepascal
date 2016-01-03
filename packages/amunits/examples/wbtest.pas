program wbtest;

{

     Try to start the program from both cli and wb.
     If from wb then click also on the icons, arg1.info,
     arg2.info and arg3.info.
     11 Nov 2000.

     Changed to use MessagBox, to show the workbench
     args create an icon for wbtest.
     28 Nov 2002.

     nils.sjoholm@mailbox.swipnet.se
}

uses wbargs, msgbox;

var
   i : integer;
   dummy : string;


Function IntToStr (I : Longint) : String;
Var
    S : String;
begin
    Str (I,S);
    IntToStr:=S;
end;


begin
  if not isconsole then begin
       dummy := 'started from wb' +#10;
       dummy := dummy + 'The Programs name is: ' + ProgramName +#10;
       dummy := dummy + 'Number of args are: ' + inttostr(WBArgCount) +#10;
       if WBArgCount > 0 then begin
          dummy := dummy + 'And the args are:' +#10;
          for i := 1 to WBArgCount do dummy := dummy + 'Arg number ' + inttostr(i) +
                        ' is: ' + GetWBArg(i) +#10;
       end;
       dummy := dummy + 'The programs name with GetWBArg(0) is: ' + GetWBArg(0);
       MessageBox('FPC WorkBench', dummy, 'Nice');
   end else begin
       writeln('started fromcli');
       writeln('The program name is: ',ProgramName);
       writeln('Number of args are: ',ParamCount);
       if ParamCount > 0 then begin
          writeln('And the args are:');
          for i := 1 to ParamCount do writeln('Arg number ',i,' is: ',ParamStr(i));
       end;
       writeln('The programs name with ParamStr(0) is: ',ParamStr(0));
   end;
end.
