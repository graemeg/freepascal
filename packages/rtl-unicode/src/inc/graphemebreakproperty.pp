unit graphemebreakproperty;

{$MODE objfpc}

interface

type
  TGraphemeBreakProperty = (
    gbpOther,
    gbpPrepend,
    gbpCR,
    gbpLF,
    gbpControl,
    gbpExtend,
    gpbRegional_Indicator,
    gbpSpacingMark,
    gbpL,
    gbpV,
    gbpT,
    gbpLV,
    gbpLVT,
    gbpE_Base,
    gbpE_Modifier,
    gbpZWJ,
    gbpGlue_After_Zwj,
    gbpE_Base_GAZ);

function GetGraphemeBreakProperty(Ch: UCS4Char): TGraphemeBreakProperty;

implementation

function GetGraphemeBreakProperty(Ch: UCS4Char): TGraphemeBreakProperty;
begin
  {$I graphemebreakproperty_code.inc}
end;

end.
