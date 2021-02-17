program monospacetext;

{$mode objfpc}{$H+}
{$codepage UTF8}

uses
  Classes, SysUtils,
  fpPDF;

var
  PDF: TPDFDocument;
  Font1, Font2, Font3, Font4: integer;
begin
  if ParamCount<1 then
    begin
    Writeln(stderr,'Usage : monospacetext <fontdir>');
    Writeln(stderr,'Needed fonts : cour.ttf, arial.ttf, verdanab.ttf consola.ttf');
    Halt(1);
    end;
  PDF := TPDFDocument.Create(nil);
  PDF.Infos.Producer := '';
  PDF.Infos.CreationDate := Now;
  PDF.Options := [poPageOriginAtTop, {poNoEmbeddedFonts,} poSubsetFont, poCompressFonts, poCompressImages];
  PDF.DefaultOrientation := ppoPortrait;
  PDF.DefaultPaperType := ptA4;
  PDF.DefaultUnitOfMeasure := uomMillimeters;
  PDF.FontDirectory := paramstr(1);
  PDF.StartDocument;
  PDF.Sections.AddSection;
  PDF.Sections[0].AddPage(PDF.Pages.AddPage);;

  //FontIndex := PDF.AddFont('Courier');
  Font1 := PDF.AddFont('cour.ttf', 'Courier New');
  Font2 := PDF.AddFont('arial.ttf', 'Arial');
  Font3 := PDF.AddFont('verdanab.ttf', 'Verdana');
  Font4 := PDF.AddFont('consola.ttf', 'Consolas');
  PDF.Pages[0].SetFont(Font1, 10);
  PDF.Pages[0].WriteText(10,10,'AEIOU-ÁÉÍÓÚ-ČŠŇŽ');
  PDF.Pages[0].WriteText(10,15,'----------------');

  PDF.Pages[0].SetFont(Font2, 10);
  PDF.Pages[0].WriteText(10,30,'AEIOU-ÁÉÍÓÚ-ČŠŇŽ');
  PDF.Pages[0].WriteText(10,35,'----------------');

  PDF.Pages[0].SetFont(Font3, 10);
  PDF.Pages[0].WriteText(10,40,'AEIOU-ÁÉÍÓÚ-ČŠŇŽ');
  PDF.Pages[0].WriteText(10,45,'----------------');

  PDF.Pages[0].SetFont(Font4, 10);
  PDF.Pages[0].WriteText(10,50,'AEIOU-ÁÉÍÓÚ-ČŠŇŽ');
  PDF.Pages[0].WriteText(10,55,'----------------');

  PDF.SaveToFile('test.pdf');
  PDF.Free;
end.

