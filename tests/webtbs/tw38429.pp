program tw38429;

{$mode objfpc}{$h+}

uses
  SysUtils, Variants, uw38429;

var
  v, d: Variant;
  I: Integer = 42;
begin
  Writeln('Test VarAsType');
  d := I;
  try
    v := VarAsType(d, varMyVar);
  except
    on e: exception do begin
      WriteLn('cast ', VarTypeAsText(VarType(d)), ' to ',VarTypeAsText(varMyVar),
              ' raises ', e.ClassName, ' with message: ', e.Message);
      Halt(1);
    end;
  end;
  WriteLn('now v is ', VarTypeAsText(VarType(v)));
  VarClear(d);
  try
    d := VarAsType(v, varInteger);
  except
    on e: exception do begin
      WriteLn('cast ', VarTypeAsText(VarType(v)), ' to ',VarTypeAsText(varInteger),
              ' raises ', e.ClassName, ' with message: ', e.Message);
      Halt(2);
    end;
  end;
  WriteLn('now d is ', VarTypeAsText(VarType(d)));

  { also test VarCast from #20849 }
  Writeln('Test VarCast');
  d := I;
  try
    VarCast(v, d, varMyVar);
  except
    on e: exception do begin
      WriteLn('cast ', VarTypeAsText(VarType(d)), ' to ',VarTypeAsText(varMyVar),
              ' raises ', e.ClassName, ' with message: ', e.Message);
      Halt(3);
    end;
  end;
  WriteLn('now v is ', VarTypeAsText(VarType(v)));
  VarClear(d);
  try
    VarCast(d, v, varInteger);
  except
    on e: exception do begin
      WriteLn('cast ', VarTypeAsText(VarType(v)), ' to ',VarTypeAsText(varInteger),
              ' raises ', e.ClassName, ' with message: ', e.Message);
      Halt(4);
    end;
  end;
  WriteLn('now d is ', VarTypeAsText(VarType(d)));
end.

