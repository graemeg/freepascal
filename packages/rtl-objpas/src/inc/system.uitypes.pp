{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2019 by Marco van de Voort
        member of the Free Pascal development team.

    Delphi compatibility unit with GUI/imaging related types.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System.UITypes;
{$mode delphi}

interface

Type  
    TColor      = Longint;
    PColor      = ^TColor;
    TColorRef   = Cardinal;
    PColorRef   = ^TColorRef;
    TAlphaColor = Cardinal;
    PAlphaColor = ^TAlphaColor;

    TColorRec = record
                 class operator := (AColor : TColor): TColorRec; inline;
                 class operator := (AColor : TColorRec): TColor; inline;
      const
      // 140 HTML colors.
      AliceBlue          = TColor($F0F8FF);
      AntiqueWhite       = TColor($FAEBD7);
      Aqua               = TColor($00FFFF);
      Aquamarine         = TColor($7FFFD4);
      Azure              = TColor($F0FFFF);
      Beige              = TColor($F5F5DC);
      Bisque             = TColor($FFE4C4);
      Black              = TColor($000000);
      BlanchedAlmond     = TColor($FFEBCD);
      Blue               = TColor($0000FF);
      BlueViolet         = TColor($8A2BE2);
      Brown              = TColor($A52A2A);
      BurlyWood          = TColor($DEB887);
      CadetBlue          = TColor($5F9EA0);
      Chartreuse         = TColor($7FFF00);
      Chocolate          = TColor($D2691E);
      Coral              = TColor($FF7F50);
      CornflowerBlue     = TColor($6495ED);
      Cornsilk           = TColor($FFF8DC);
      Crimson            = TColor($DC143C);
      Cyan               = TColor($00FFFF);
      DarkBlue           = TColor($00008B);
      DarkCyan           = TColor($008B8B);
      DarkGoldenRod      = TColor($B8860B);
      DarkGray           = TColor($A9A9A9);
      DarkGreen          = TColor($006400);
      DarkGrey           = TColor($A9A9A9);
      DarkKhaki          = TColor($BDB76B);
      DarkMagenta        = TColor($8B008B);
      DarkOliveGreen     = TColor($556B2F);
      DarkOrange         = TColor($FF8C00);
      DarkOrchid         = TColor($9932CC);
      DarkRed            = TColor($8B0000);
      DarkSalmon         = TColor($E9967A);
      DarkSeaGreen       = TColor($8FBC8F);
      DarkSlateBlue      = TColor($483D8B);
      DarkSlateGray      = TColor($2F4F4F);
      DarkSlateGrey      = TColor($2F4F4F);
      DarkTurquoise      = TColor($00CED1);
      DarkViolet         = TColor($9400D3);
      DeepPink           = TColor($FF1493);
      DeepSkyBlue        = TColor($00BFFF);
      DimGray            = TColor($696969);
      DimGrey            = TColor($696969);
      DodgerBlue         = TColor($1E90FF);
      FireBrick          = TColor($B22222);
      FloralWhite        = TColor($FFFAF0);
      ForestGreen        = TColor($228B22);
      Fuchsia            = TColor($FF00FF);
      Gainsboro          = TColor($DCDCDC);
      GhostWhite         = TColor($F8F8FF);
      Gold               = TColor($FFD700);
      GoldenRod          = TColor($DAA520);
      Gray               = TColor($808080);
      Green              = TColor($008000);
      GreenYellow        = TColor($ADFF2F);
      Grey               = TColor($808080);
      HoneyDew           = TColor($F0FFF0);
      HotPink            = TColor($FF69B4);
      IndianRed          = TColor($CD5C5C);
      Indigo             = TColor($4B0082);
      Ivory              = TColor($FFFFF0);
      Khaki              = TColor($F0E68C);
      Lavender           = TColor($E6E6FA);
      LavenderBlush      = TColor($FFF0F5);
      LawnGreen          = TColor($7CFC00);
      LemonChiffon       = TColor($FFFACD);
      LightBlue          = TColor($ADD8E6);
      LightCoral         = TColor($F08080);
      LightCyan          = TColor($E0FFFF);
      LightGoldenRodYellow    = TColor($FAFAD2);
      LightGray          = TColor($D3D3D3);
      LightGreen         = TColor($90EE90);
      LightGrey          = TColor($D3D3D3);
      LightPink          = TColor($FFB6C1);
      LightSalmon        = TColor($FFA07A);
      LightSeaGreen      = TColor($20B2AA);
      LightSkyBlue       = TColor($87CEFA);
      LightSlateGray     = TColor($778899);
      LightSlateGrey     = TColor($778899);
      LightSteelBlue     = TColor($B0C4DE);
      LightYellow        = TColor($FFFFE0);
      Lime               = TColor($00FF00);
      LimeGreen          = TColor($32CD32);
      Linen              = TColor($FAF0E6);
      Magenta            = TColor($FF00FF);
      Maroon             = TColor($800000);
      MediumAquaMarine   = TColor($66CDAA);
      MediumBlue         = TColor($0000CD);
      MediumOrchid       = TColor($BA55D3);
      MediumPurple       = TColor($9370DB);
      MediumSeaGreen     = TColor($3CB371);
      MediumSlateBlue    = TColor($7B68EE);
      MediumSpringGreen  = TColor($00FA9A);
      MediumTurquoise    = TColor($48D1CC);
      MediumVioletRed    = TColor($C71585);
      MidnightBlue       = TColor($191970);
      MintCream          = TColor($F5FFFA);
      MistyRose          = TColor($FFE4E1);
      Moccasin           = TColor($FFE4B5);
      NavajoWhite        = TColor($FFDEAD);
      Navy               = TColor($000080);
      OldLace            = TColor($FDF5E6);
      Olive              = TColor($808000);
      OliveDrab          = TColor($6B8E23);
      Orange             = TColor($FFA500);
      OrangeRed          = TColor($FF4500);
      Orchid             = TColor($DA70D6);
      PaleGoldenRod      = TColor($EEE8AA);
      PaleGreen          = TColor($98FB98);
      PaleTurquoise      = TColor($AFEEEE);
      PaleVioletRed      = TColor($DB7093);
      PapayaWhip         = TColor($FFEFD5);
      PeachPuff          = TColor($FFDAB9);
      Peru               = TColor($CD853F);
      Pink               = TColor($FFC0CB);
      Plum               = TColor($DDA0DD);
      PowderBlue         = TColor($B0E0E6);
      Purple             = TColor($800080);
      RebeccaPurple      = TColor($663399);
      Red                = TColor($FF0000);
      RosyBrown          = TColor($BC8F8F);
      RoyalBlue          = TColor($4169E1);
      SaddleBrown        = TColor($8B4513);
      Salmon             = TColor($FA8072);
      SandyBrown         = TColor($F4A460);
      SeaGreen           = TColor($2E8B57);
      SeaShell           = TColor($FFF5EE);
      Sienna             = TColor($A0522D);
      Silver             = TColor($C0C0C0);
      SkyBlue            = TColor($87CEEB);
      SlateBlue          = TColor($6A5ACD);
      SlateGray          = TColor($708090);
      SlateGrey          = TColor($708090);
      Snow               = TColor($FFFAFA);
      SpringGreen        = TColor($00FF7F);
      SteelBlue          = TColor($4682B4);
      Tan                = TColor($D2B48C);
      Teal               = TColor($008080);
      Thistle            = TColor($D8BFD8);
      Tomato             = TColor($FF6347);
      Turquoise          = TColor($40E0D0);
      Violet             = TColor($EE82EE);
      Wheat              = TColor($F5DEB3);
      White              = TColor($FFFFFF);
      WhiteSmoke         = TColor($F5F5F5);
      Yellow             = TColor($FFFF00);
      YellowGreen        = TColor($9ACD32);
      var
        case Integer of
          0:  {$IFDEF ENDIAN_BIG}
              (A,B,G,R : Byte);
              {$else}
              (R,G,B,A : Byte);
              {$ENDIF}
          1: (Color : TColor);
      end;

// copied from Lazutils version
///////////////////////////////

type
  // Message dialog related
  TMsgDlgType    = (mtWarning, mtError, mtInformation, mtConfirmation, mtCustom);
  TMsgDlgBtn     = (mbYes, mbNo, mbOK, mbCancel, mbAbort, mbRetry, mbIgnore,
                    mbAll, mbNoToAll, mbYesToAll, mbHelp, mbClose);
  TMsgDlgButtons = set of TMsgDlgBtn;

  // ModalResult
  TModalResult = low(Integer)..high(Integer);
  PModalResult = ^TModalResult;

const
  // Used for ModalResult
  mrNone = 0;
  mrOK = mrNone + 1;
  mrCancel = mrNone + 2;
  mrAbort = mrNone + 3;
  mrRetry = mrNone + 4;
  mrIgnore = mrNone + 5;
  mrYes = mrNone + 6;
  mrNo = mrNone + 7;
  mrAll = mrNone + 8;
  mrNoToAll = mrNone + 9;
  mrYesToAll = mrNone + 10;
  mrClose = mrNone + 11;
  mrLast = mrClose;

  // String representation of ModalResult values
  ModalResultStr: array[mrNone..mrLast] of shortstring = (
    'mrNone',
    'mrOk',
    'mrCancel',
    'mrAbort',
    'mrRetry',
    'mrIgnore',
    'mrYes',
    'mrNo',
    'mrAll',
    'mrNoToAll',
    'mrYesToAll',
    'mrClose');

implementation

class operator TColorRec.:= (AColor : TColor): TColorRec;
begin
  result.Color:=AColor;
end;

class operator TColorRec.:= (AColor : TColorRec): TColor;
begin
  result:=AColor.Color;
end;

end.
