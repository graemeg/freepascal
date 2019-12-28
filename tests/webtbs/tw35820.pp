program BugExample;

{$mode ObjFPC}
{$GOTO ON}

type SubRange = 1..3;

  procedure Blah(const I: SubRange); inline;
  var
    B: Boolean = True;
  label
    Top;
  begin
    Top:
      case I of
        1:
          WriteLn(2);
        2:
          if B then
          begin
            B := False;
            WriteLn('Resetting!');
            goto Top;
          end
          else
            WriteLn(4);
        3:
          WriteLn(6);
      end;
  end;

  procedure DoIt;
  begin
    Blah(1);
    Blah(2);
    Blah(3);
  end;

begin
  DoIt;
end.
