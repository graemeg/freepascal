<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
unit tvalc;

interface
const
  HasErrors : boolean = false;
  Silent : boolean = true;
  CheckVal : boolean = true;
  SuccessCount : longint = 0;
  FailCount : longint = 0;

type
  TCharSet = set of char;
const
  ValidNumeralsBase2 : TCHarSet = ['0'..'1'];
  ValidNumeralsBase8 : TCHarSet = ['0'..'7'];
  ValidNumeralsBase10 : TCHarSet = ['0'..'9'];
  ValidNumeralsBase16 : TCHarSet = ['0'..'9','a'..'f','A'..'F'];
  SpecialCharsFirst : TCharSet = [' ',#9,'x','X','$','&','%','+','-'];
  SpecialCharsSecond : TCharSet = [#0];

type

  ValTestType =
  (ValShouldFail,
   ValShouldSucceed,
   ValShouldSucceedAfterRemovingTrail);


function Display(const s : string) : string;

implementation

function Display(const s : string) : string;
var
  res,ordval : string;
  i : longint;
  quoted : boolean;
begin
  res:='"';
  quoted:=false;
  for i:=1 to length(s) do
    if ord(s[i])<32 then
      begin
        if quoted then
          res:=res+'''';
        str(ord(s[i]),ordval);
        res:=res+'#'+ordval;
        quoted:=false;
      end
    else
      begin
        if not quoted then
          res:=res+'''';
        quoted:=true;
        res:=res+s[i];
      end;
  if quoted then
    res:=res+'''';
  res:=res+'"';
  Display:=res;
end;

end.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
unit tvalc;

interface
const
  HasErrors : boolean = false;
  Silent : boolean = true;
  CheckVal : boolean = true;
  SuccessCount : longint = 0;
  FailCount : longint = 0;

type
  TCharSet = set of char;
const
  ValidNumeralsBase2 : TCHarSet = ['0'..'1'];
  ValidNumeralsBase8 : TCHarSet = ['0'..'7'];
  ValidNumeralsBase10 : TCHarSet = ['0'..'9'];
  ValidNumeralsBase16 : TCHarSet = ['0'..'9','a'..'f','A'..'F'];
  SpecialCharsFirst : TCharSet = [' ',#9,'x','X','$','&','%','+','-'];
  SpecialCharsSecond : TCharSet = [#0];

type

  ValTestType =
  (ValShouldFail,
   ValShouldSucceed,
   ValShouldSucceedAfterRemovingTrail);


function Display(const s : string) : string;

implementation

function Display(const s : string) : string;
var
  res,ordval : string;
  i : longint;
  quoted : boolean;
begin
  res:='"';
  quoted:=false;
  for i:=1 to length(s) do
    if ord(s[i])<32 then
      begin
        if quoted then
          res:=res+'''';
        str(ord(s[i]),ordval);
        res:=res+'#'+ordval;
        quoted:=false;
      end
    else
      begin
        if not quoted then
          res:=res+'''';
        quoted:=true;
        res:=res+s[i];
      end;
  if quoted then
    res:=res+'''';
  res:=res+'"';
  Display:=res;
end;

end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
