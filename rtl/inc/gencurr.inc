{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2007 by Several contributors

    Generic mathematical routines (on type currency)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

{$ifdef FPC_CURRENCY_IS_INT64}

    function trunc(c : currency) : int64; {$ifdef systeminline} inline; {$endif}
      begin
        { the type conversion includes dividing by 10000 }
        result := int64(c)
      end;

{$ifndef cpujvm}
    function trunc(c : comp) : int64; {$ifdef systeminline} inline; {$endif}
{$else not cpujvm}
    function trunc_comp(c : comp) : int64; {$ifdef systeminline} inline; {$endif}
{$endif cpujvm}
      begin
        result := c
      end;

{$ifndef FPUNONE}
    function round(c : currency) : int64;
      var
        rem, absrem: currency;
      begin
        result := int64(c);
        rem := c - currency(result);
        case softfloat_rounding_mode of
          rmNearest:
            begin
              absrem := abs(rem);
              if (absrem > 0.5) or
                 ((absrem = 0.5) and
                  odd(result)) then
                if (rem > 0) then
                  inc(result)
                else
                  dec(result)
            end;
          rmDown:
            begin
              if rem < 0 then
                dec(result);
            end;
          rmUp:
            begin
              if rem > 0 then
                inc(result);
            end;
          rmTruncate:
            begin
              // result is already ok
            end;
        end;
      end;
{$endif FPUNONE}


{$ifndef cpujvm}
    function round(c : comp) : int64; {$ifdef systeminline} inline; {$endif}
{$else not cpujvm}
    function round_comp(c : comp) : int64; {$ifdef systeminline} inline; {$endif}
{$endif cpujvm}
      begin
        result := c
      end;

{$endif FPC_CURRENCY_IS_INT64}




