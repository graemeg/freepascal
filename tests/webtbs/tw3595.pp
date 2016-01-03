{ Source provided for Free Pascal Bug Report 3595 }
{ Submitted by "Martin Schreiber" on  2005-01-24 }
{ e-mail:  }
program project1;

{$ifdef FPC} {$mode objfpc}{$H+} {$endif}

uses
  Classes,sysutils;

type
 ttest = class(tcomponent)
  private
   fwstr: widestring;
  published
   property wstr: widestring read fwstr write fwstr;
 end;

const
 textfilename = 'test.txt';

var
 test1,test2: ttest;
 memstream: tmemorystream;
 filestream: tfilestream;

function widestringtocharinfo(const str: widestring): string;
var
 int1: integer;
begin
 result:= '';
 for int1:= 1 to length(str) do begin
  result:= result + '#' + inttostr(ord(str[int1]));
 end;
end;

begin
 test1:= ttest.create(nil);
 test2:= ttest.create(nil);
 with test1 do begin
  setlength(fwstr,3);
  fwstr[1]:= widechar(255);
  fwstr[2]:= widechar(256);
  fwstr[3]:= widechar(257);
  fwstr:= fwstr + #0#1'abcde'#127#128#129;
 end;

 filestream:= tfilestream.create(textfilename,fmcreate);
 memstream:= tmemorystream.create;
 memstream.writecomponent(test1);
 memstream.position:= 0;
 objectbinarytotext(memstream,filestream);
 memstream.free;
 filestream.free;


// try
  filestream:= tfilestream.create(textfilename,fmopenread);
  memstream:= tmemorystream.create;
//  try
   objecttexttobinary(filestream,memstream);
   writeln('objecttexttobinary OK');
   try
    memstream.position:= 0;
    memstream.readcomponent(test2);
    writeln('object reading OK');
    if test1.wstr = test2.wstr then begin
     writeln('data OK');
    end
    else begin
     writeln('data wrong. expected:');
     writeln(widestringtocharinfo(test1.wstr));
     writeln('actual:');
     writeln(widestringtocharinfo(test2.wstr));
     halt(1);
    end;
   except
    on e: exception do begin
     writeln('object reading fails: '+ e.message);
     halt(1);
    end;
   end;
{  except
   on e: exception do begin
    writeln('objecttexttobinary fails: '+e.message);
    
    halt(1);
   end;
  end;
}  filestream.free;
  memstream.free;
{
except
  writeln('file '+textfilename+' not found.');
  halt(1);
 end;
} test1.free;
 test2.free;
 deletefile(textfilename);
end.
