program metautf16;

{$codepage utf-8}

uses sysutils,fpPDF;

var
  D:TpdfDocument;
  S:TPdfSection;
  P:TPdfPage;

begin
  D:=TpdfDocument.Create(nil);
  try
    D.Infos.Title := 'Урывак з паэмы "Новая Зямля"';
    D.Infos.Author := 'Якуб Колас';
    D.Infos.Producer := 'fcl-pdf';
    D.Infos.ApplicationName := 'нейкі тэст';
    D.Infos.CreationDate := Now;
    D.Infos.KeyWords:='fcl-pdf report';

    D.Options := [poPageOriginAtTop,poSubsetFont,poCompressFonts,poCompressImages,poUseImageTransparency,poUTF16Info];

    D.StartDocument;
    D.AddFont('fonts/FreeSans.ttf','FreeSans');

    
    S:=D.Sections.AddSection;      
   
    P:=D.Pages.AddPage;
    P.PaperType := ptA4;
    P.UnitOfMeasure := uomPixels;
    P.Orientation:=ppoPortrait;
    S.AddPage(P);

    P.SetFont(0,10);
    P.WriteText(100,100,'Мой родны кут,');
    P.WriteText(100,150,'Як ты мне мілы');
    P.WriteText(100,200,'Забыць цябе');
    P.WriteText(100,250,'Не маю сілы');
  finally
    D.SaveToFile('test.pdf');
    D.Free;
  end;
end.