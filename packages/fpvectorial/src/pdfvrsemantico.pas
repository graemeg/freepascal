unit pdfvrsemantico;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, pdfvrlexico, fpvectorial;

type

  { AnSemantico }

  AnSemantico = class
  public
    FPointSeparator, FCommaSeparator: TFormatSettings;
    close_path_x: String;
    close_path_y: String;
    cm_a, cm_b, cm_c, cm_d, cm_e, cm_f: Real; // coordinate spaces constants
    function StringToFloat(AStr: string): Double;
    function generate(c: Command; AData: TvVectorialDocument): String;
    function convert(x: String; y: String; Axis: Char): String;
    function startMachine(): String;
    function endMachine(): String;
    constructor Create;
  end;

implementation

{ PDF doesn't seam very consistent when it comes to using commas or
  points as decimal separator, so we just try both }
function AnSemantico.StringToFloat(AStr: string): Double;
begin
  if Pos('.', AStr) > 0 then Result := StrToFloat(AStr, FPointSeparator)
  else Result := StrToFloat(AStr, FCommaSeparator);
end;

function AnSemantico.generate(c: Command; AData: TvVectorialDocument): String;
var
  enter_line : String;
begin
  {$ifdef FPVECTORIALDEBUG}
  WriteLn(':> AnSemantico.generate');
  {$endif}

  enter_line:= LineEnding; //chr(13) + chr(10); // CR and LF

  if ((c.code = cc_H_CLOSE_PATH) or (c.code = cc_hS_CLOSE_AND_END_PATH)) then // command h or s
  begin
    c.cord_x:=close_path_x;
    c.cord_y:=close_path_y;
  end;

  if ((c.code <> cc_H_CLOSE_PATH) and (c.code <> cc_hS_CLOSE_AND_END_PATH)) then // close path already converted
  begin
     if ((c.code = cc_m_START_PATH) or (c.code = cc_l_ADD_LINE_TO_PATH)) then
     begin
       //WriteLn(':: anSemantico.generate convert code ', Integer(c.code));
       c.cord_x := convert(c.cord_x,c.cord_y,'x');
       c.cord_y := convert(c.cord_x,c.cord_y,'y');
     end;
     if ((c.code = cc_c_BEZIER_TO_X_Y_USING_X2_Y2_AND_X3_Y3)) then
     begin
       //WriteLn(':: anSemantico.generate convert code ', Integer(c.code));
       c.cord_x := convert(c.cord_x,c.cord_y,'x');
       c.cord_y := convert(c.cord_x,c.cord_y,'y');
       c.cord_x2 := convert(c.cord_x2,c.cord_y2,'x');
       c.cord_y2 := convert(c.cord_x2,c.cord_y2,'y');
       c.cord_x3 := convert(c.cord_x3,c.cord_y3,'x');
       c.cord_y3 := convert(c.cord_x3,c.cord_y3,'y');
     end;
  end;

  case c.code of
  cc_m_START_PATH: // command m
  begin
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.generate Estado 1 EndPath StartPath');
    {$endif}
    // Result:='G01' + ' ' + 'X' + c.cord_x + ' ' + 'Y' + c.cord_y + enter_line +
    //            'G01 Z50 // Abaixa a cabeça de gravação';

    // Correcao para programas de desenho que geram um novo inicio no
    // fim do desenho, terminamos qualquer desenho inacabado
    AData.EndPath();
    AData.StartPath(StringToFloat(c.cord_x), StringToFloat(c.cord_y));

    close_path_x:=c.cord_x;
    close_path_y:=c.cord_y;
  end;
  cc_l_ADD_LINE_TO_PATH: // command l
  begin
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.generate Estado 2 AddPointToPath');
    {$endif}
    // Result:='G01' + ' ' + 'X' + c.cord_x + ' ' +  'Y' + c.cord_y;

    AData.AddLineToPath(StringToFloat(c.cord_x), StringToFloat(c.cord_y));
  end;
  cc_h_CLOSE_PATH: // command h
  begin
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.generate Estado 3 AddPointToPath');
    {$endif}
    //Result:='G01' + ' ' + 'X' + c.cord_x + ' ' +  'Y' + c.cord_y;

    AData.AddLineToPath(StringToFloat(c.cord_x), StringToFloat(c.cord_y));
  end;
  cc_S_END_PATH: // command S
  begin
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.generate Estado 4 EndPath');
    {$endif}
    // Result:='G01 Z0 // Sobe a cabeça de gravação' + enter_line;
    AData.EndPath();
  end;
  cc_hS_CLOSE_AND_END_PATH: // command s
  begin
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.generate Estado 5 AddPoint EndPath');
    {$endif}
    //Result:='G01' + ' ' + 'X' + c.cord_x + ' ' +  'Y' + c.cord_y + enter_line
    //       +'G01 Z0 // Sobe a cabeça de gravação' + enter_line;

    AData.AddLineToPath(StringToFloat(c.cord_x), StringToFloat(c.cord_y));
    AData.EndPath();
  end;
  cc_c_BEZIER_TO_X_Y_USING_X2_Y2_AND_X3_Y3: // command c
  begin
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.generate Estado 6 Bezier');
    {$endif}
    //Result:='G01' + ' ' + 'X' + c.cord_x + ' ' +  'Y' + c.cord_y + enter_line
    //       +'G01 Z0 // Sobe a cabeça de gravação' + enter_line;

    AData.AddBezierToPath(
      StringToFloat(c.cord_x3), StringToFloat(c.cord_y3),
      StringToFloat(c.cord_x2), StringToFloat(c.cord_y2),
      StringToFloat(c.cord_x), StringToFloat(c.cord_y)
      );
  end;
  cc_CONCATENATE_MATRIX: // command cm
  begin
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.cc_CONCATENATE_MATRIX');
    {$endif}

    cm_a := StringToFloat(c.cord_x3);
    cm_b := StringToFloat(c.cord_y3);
    cm_c := StringToFloat(c.cord_x2);
    cm_d := StringToFloat(c.cord_y2);
    cm_e := StringToFloat(c.cord_x);
    cm_f := StringToFloat(c.cord_y);
  end;
  cc_RESTORE_MATRIX: // command Q
  begin
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.cc_RESTORE_MATRIX');
    {$endif}

    cm_a:=1;
    cm_b:=0;
    cm_c:=0;
    cm_d:=1;
    cm_e:=0;
    cm_f:=0;
  end;
  else
    {$ifdef FPVECTORIALDEBUG}
    WriteLn(':> AnSemantico.generate Estado ELSE');
    {$endif}
    Result:=c.my_operator;
  end;
end;

function AnSemantico.convert(x: String; y: String; Axis: Char): String;
begin
  {$ifdef FPVECTORIALDEBUG}
  WriteLn(':> AnSemantico.convert');
  {$endif}
  // convert from 1/72 inch to milimeters and change axis if necessary

  if (Axis = 'y') then
  begin
       // y' = b * x + d * y + f
       Result:=FloatToStr((cm_b*StringToFloat(x)+cm_d*StringToFloat(y)+cm_f)*(25.40/72));
  end
  else
  // Axis = 'x'
  begin
       // x' = a * x + c * y + e
       Result:=FloatToStr((cm_a*StringToFloat(x)+cm_c*StringToFloat(y)+cm_e)*(25.40/72));
  end;
end;

function AnSemantico.startMachine(): String;
var
  enter_line : String;
begin
  {$ifdef FPVECTORIALDEBUG}
  WriteLn(':> AnSemantico.startMachine');
  {$endif}
  enter_line:=chr(13) + chr(10); // CR and LF

  Result:='M216 // Ligar monitor de carga' + enter_line +
          'G28 // Ir rapidamente para posição inicial' + enter_line +
          'G00' + enter_line;
end;

function AnSemantico.endMachine(): String;
var
  enter_line : String;
begin
  {$ifdef FPVECTORIALDEBUG}
  WriteLn(':> AnSemantico.endMachine');
  {$endif}
  enter_line:=chr(13) + chr(10); // CR and LF

  Result:='M30 // Parar o programa e retornar para posição inicial' + enter_line +
          'M215 // Desligar monitor de carga' + enter_line;
end;

constructor AnSemantico.Create;
begin
  inherited Create;

  cm_a:=1;
  cm_b:=0;
  cm_c:=0;
  cm_d:=1;
  cm_e:=0;
  cm_f:=0;

  // Format seetings to convert a string to a float
  FPointSeparator := DefaultFormatSettings;
  FPointSeparator.DecimalSeparator := '.';
  FPointSeparator.ThousandSeparator := '#';// disable the thousand separator
  FCommaSeparator := DefaultFormatSettings;
  FCommaSeparator.DecimalSeparator := ',';
  FCommaSeparator.ThousandSeparator := '#';// disable the thousand separator
end;

end.

