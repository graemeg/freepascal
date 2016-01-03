PROGRAM GetFontAsltest;

uses easyasl,msgbox,amigautils;

{
   An example to get a font with easyasl.
   24 Jan 2000.

   nils.sjoholm@mailbox.swipnet.se
}

VAR
    myfont : tFPCFontInfo;
    dummy  : BOOLEAN;


BEGIN

    dummy := GetFontAsl('Pick a font',myfont,NIL);
    IF dummy THEN BEGIN
       MessageBox('FPC Pascal Request',
                  'You picked as font   :' + myfont.nfi_Name + #10 +
                  'The fontsize is      :' + longtostr(myfont.nfi_Size) + #10 +
                  'The fontstyle is     :' + longtostr(myfont.nfi_Style) + #10 +
                  'The flags are set to :' + longtostr(myfont.nfi_Flags) + #10 +
                  'Frontpen is number   :' + longtostr(myfont.nfi_FrontPen) + #10 +
                  'And as the backpen   :' + longtostr(myfont.nfi_BackPen) + #10 +
                  'And finally drawmode :' + longtostr(myfont.nfi_DrawMode),
                  'Nice font!');
    END ELSE
       MessageBox('FPC Pascal request','You didn''t pick a font','Why not?');
END.
