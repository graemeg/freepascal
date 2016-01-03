{
Writes AvisoCNC G-Code

License: The same modified LGPL as the Free Pascal RTL
         See the file COPYING.modifiedLGPL for more details

AUTHORS: Felipe Monteiro de Carvalho
         Pedro Sol Pegorini L de Lima
}
unit avisocncgcodewriter;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,
  fpvectorial;

type
  { TvAvisoCNCGCodeWriter }

  TvAvisoCNCGCodeWriter = class(TvCustomVectorialWriter)
  public
    { General reading methods }
    procedure WriteToStrings(AStrings: TStrings; AData: TvVectorialDocument); override;
  end;

implementation

{ TvGCodeVectorialWriter }

procedure TvAvisoCNCGCodeWriter.WriteToStrings(AStrings: TStrings;
  AData: TvVectorialDocument);
var
  i, j: Integer;
  Str: string;
  APath: TPath;
  CurSegment: T2DSegment;
  Cur3DSegment: T3DSegment;
  Cur2DBezierSegment: T2DBezierSegment;
  Cur3DBezierSegment: T3DBezierSegment;
begin
  AStrings.Clear;

  AStrings.Add('M216 // Ligar monitor de carga');
  AStrings.Add('G28 // Ir rapidamente para posição inicial');
  AStrings.Add('G00');

  // itera por todos os itens
  for i := 0 to AData.GetPathCount - 1 do
  begin
    APath := AData.GetPath(i);

    // levanta a broca
    AStrings.Add('P01 // Sobe a cabeça de gravação');
    // vai para o ponto inicial
    CurSegment := T2DSegment(APath.Points);
    AStrings.Add(Format('G01 X%f Y%f',
      [CurSegment.X, CurSegment.Y]));
    AStrings.Add('P02 // Abaixa a cabeça de gravação');

    for j := 1 to APath.Len - 1 do
    begin
      CurSegment := T2DSegment(CurSegment.Next);
      case CurSegment.SegmentType of
      st2DLine: AStrings.Add(Format('G01 X%f Y%f',
         [CurSegment.X, CurSegment.Y]));
      st3DLine:
      begin
        Cur3DSegment := T3DSegment(CurSegment);
        AStrings.Add(Format('G01 X%f Y%f Z%f',
         [Cur3DSegment.X, Cur3DSegment.Y, Cur3DSegment.Z]));
      end;
      st2DBezier:
      begin
        Cur2DBezierSegment := T2DBezierSegment(CurSegment);
        AStrings.Add(Format('B02 X%f Y%f X%f Y%f X%f Y%f',
         [Cur2DBezierSegment.X2, Cur2DBezierSegment.Y2,
          Cur2DBezierSegment.X3, Cur2DBezierSegment.Y3,
          Cur2DBezierSegment.X, Cur2DBezierSegment.Y]));
      end;
      st3DBezier:
      begin
        Cur3DBezierSegment := T3DBezierSegment(CurSegment);
        AStrings.Add(Format('B03 X%f Y%f Z%f X%f Y%f Z%f X%f Y%f Z%f',
         [Cur3DBezierSegment.X2, Cur3DBezierSegment.Y2, Cur3DBezierSegment.Z2,
          Cur3DBezierSegment.X3, Cur3DBezierSegment.Y3, Cur3DBezierSegment.Z3,
          Cur3DBezierSegment.X, Cur3DBezierSegment.Y, Cur3DBezierSegment.Z]));
      end;
      end;
    end;
  end;

  AStrings.Add('P01 // Sobe a cabeça de gravação');
  AStrings.Add('M30 // Parar o programa e retornar para posição inicial');
  AStrings.Add('M215 // Desligar monitor de carga');
end;

initialization

  RegisterVectorialWriter(TvAvisoCNCGCodeWriter, vfGCodeAvisoCNCPrototipoV5);

end.

