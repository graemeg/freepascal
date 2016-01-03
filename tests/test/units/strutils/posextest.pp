{$mode objfpc} {$h+}
{$hints on}
{$warnings on}

uses strutils;

var doexit : integer =0;

procedure posTest(const substr,str:ansistring;start:integer;shouldberes:integer;testnr:integer);

var res : integer;

begin
  res:=posex(substr,str,start);
  if res<>shouldberes then
   begin
     writeln('test ',testnr:5,' resulted in ',res:5,' should be ',shouldberes);
     doexit:=1;
   end;
  if res>0 then
   begin
     if copy(str,res,length(substr))<>substr then
      begin
        doexit:=1;
        writeln('test ',testnr,' doesn''t match search phrase');
      end;
   end;
end; 

const 
  S = 'Start'+#0#1+'BaseLevel'+#0#2+'Sublevel1'+#0#2+'Sublevel2'+#0#1+'LastOne';
  Sub = 'LastOne';

  s2 = '1234one8901one';
  s3 = '1234one8901on';
  s4 = '1234on234on3';
begin
  postest(sub,s,1,41,1);
  postest('One',s,1,45,2);
  postest('Start',s,1,1,3);
  postest('one',s2,1,5,4);
  postest('one',s2,6,12,5);
  postest('one',s3,6,0,6);
  postest('one',s3,0,0,7);
  postest('one',s2,14,0,8);
  postest('One',s2,1,0,9); // test if compare is case sensitive
  postest('one',s4,1,0,10); // test if compare is case sensitive

  if doexit>0 then
    halt(1);
end.
