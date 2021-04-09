type
       TSimpleEnum = (seOne, seTwo);
       TSimpleSet = set of TSimpleEnum;

       TRecordWithSet = record
               TheSet  :       TSimpleSet;
       end;

function FirstFunc:TRecordWithSet;
begin
       FirstFunc.TheSet := [];

       //below would work fine
       //FirstFunc.TheSet := FirstFunc.TheSet + [seOne];

       //below line causes error "Fatal: Internal error 200405022"
       Include(FirstFunc.TheSet, seOne);
	   if not(seOne in FirstFunc.TheSet) then
	     halt(1);
end;

//absolute variable overlaying Result doesn't help
function SecondFunc:TRecordWithSet;
var
       LocalAbs        : TRecordWithSet absolute SecondFunc;
begin
       LocalAbs.TheSet := [];
       //below line would cause same error
       Include(LocalAbs.TheSet, seOne);
	   if not(seOne in LocalAbs.TheSet) then
	     halt(1);
end;

var
       Collected       : TRecordWithSet;
begin
       Collected := FirstFunc;
end.
