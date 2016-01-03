{$MODE objfpc}

{$I endian.pas}

Uses
  SysUtils, ptc;

Const
  destXSize = {480}320;
  destYSize = {300}200;

Var
  image : TPTCSurface;
  surface : TPTCSurface;
  format : TPTCFormat;
  TestNum : Integer;

Function fb(q : int32) : Integer;

Begin
  fb := 0;
  While (q And 1) = 0 Do
  Begin
    Inc(fb);
    q := q Shr 1;
  End;
End;

Function nb(q : int32) : Integer;

Begin
  nb := 0;
  While q <> 0 Do
  Begin
    Inc(nb);
    q := q And (q - 1);
  End;
End;

Procedure generic(src, dest : TPTCSurface);

Var
  X, Y : Integer;
  XSize, YSize : Integer;
  r, g, b : int32;
  pix : int32;
  Psrc, Pdest : Pchar8;
  srcbits : Integer;
  Srmask, Sgmask, Sbmask : int32;
  Srmasknb, Sgmasknb, Sbmasknb : Integer;
  Srmaskfb, Sgmaskfb, Sbmaskfb : Integer;
  destbits : Integer;
  Drmask, Dgmask, Dbmask : int32;
  Drmasknb, Dgmasknb, Dbmasknb : Integer;
  Drmaskfb, Dgmaskfb, Dbmaskfb : Integer;

Begin
  XSize := dest.width;
  YSize := dest.height;
  
  srcbits := src.format.bits;
  Srmask := src.format.r;
  Sgmask := src.format.g;
  Sbmask := src.format.b;
  Srmasknb := nb(Srmask);
  Sgmasknb := nb(Sgmask);
  Sbmasknb := nb(Sbmask);
  Srmaskfb := fb(Srmask);
  Sgmaskfb := fb(Sgmask);
  Sbmaskfb := fb(Sbmask);
  
  destbits := dest.format.bits;
  Drmask := dest.format.r;
  Dgmask := dest.format.g;
  Dbmask := dest.format.b;
  Drmasknb := nb(Drmask);
  Dgmasknb := nb(Dgmask);
  Dbmasknb := nb(Dbmask);
  Drmaskfb := fb(Drmask);
  Dgmaskfb := fb(Dgmask);
  Dbmaskfb := fb(Dbmask);
  
{  Writeln(Srmasknb, ' ', Drmasknb);}
  
  Psrc := src.lock;
  Pdest := dest.lock;
  
  For Y := 0 To YSize - 1 Do
    For X := 0 To XSize - 1 Do
    Begin
      Case srcbits Of
        32 : Begin
          pix := (Pint32(Psrc))^;
          Inc(Psrc, 4);
	End;
	24 : Begin
	  {$IFDEF FPC_LITTLE_ENDIAN}
	    pix := (Psrc^) Or ((Psrc + 1)^ Shl 8) Or ((Psrc + 2)^ Shl 16);
	  {$ELSE FPC_LITTLE_ENDIAN}
	    pix := (Psrc^ Shl 16) Or ((Psrc + 1)^ Shl 8) Or ((Psrc + 2)^);
	  {$ENDIF FPC_LITTLE_ENDIAN}
	  Inc(Psrc, 3);
	End;
	16 : Begin
	  pix := (Pshort16(Psrc))^;
	  Inc(Psrc, 2);
	End;
	8 : Begin
	  pix := Psrc^;
	  Inc(Psrc);
	End;
      End;
      
      r := pix And Srmask;
      g := pix And Sgmask;
      b := pix And Sbmask;
      r := r Shr Srmaskfb;
      g := g Shr Sgmaskfb;
      b := b Shr Sbmaskfb;
      
      If (Drmasknb - Srmasknb) >= 0 Then
        r := r Shl (Drmasknb - Srmasknb)
      Else
        r := r Shr (Srmasknb - Drmasknb);
      If (Dgmasknb - Sgmasknb) >= 0 Then
        g := g Shl (Dgmasknb - Sgmasknb)
      Else
        g := g Shr (Sgmasknb - Dgmasknb);
      If (Dbmasknb - Sbmasknb) >= 0 Then
        b := b Shl (Dbmasknb - Sbmasknb)
      Else
        b := b Shr (Sbmasknb - Dbmasknb);
      
      r := r Shl Drmaskfb;
      g := g Shl Dgmaskfb;
      b := b Shl Dbmaskfb;
      pix := r Or g Or b;
      
      Case destbits Of
        32 : Begin
          (Pint32(Pdest))^ := pix;
          Inc(Pdest, 4);
	End;
	24 : Begin
	  {$IFDEF FPC_LITTLE_ENDIAN}
	    Pdest^ := pix And $FF;
	    (Pdest + 1)^ := (pix Shr 8) And $FF;
	    (Pdest + 2)^ := (pix Shr 16) And $FF;
	  {$ELSE FPC_LITTLE_ENDIAN}
	    Pdest^ := (pix Shr 16) And $FF;
	    (Pdest + 1)^ := (pix Shr 8) And $FF;
	    (Pdest + 2)^ := pix And $FF;
	  {$ENDIF FPC_LITTLE_ENDIAN}
	  Inc(Pdest, 3);
	End;
	16 : Begin
	  (Pshort16(Pdest))^ := pix;
	  Inc(Pdest, 2);
	End;
	8 : Begin
	  Pdest^ := pix;
	  Inc(Pdest);
	End;
      End;
    End;
  src.unlock;
  dest.unlock;
End;

Procedure test(sbits : Integer; sr, sg, sb : int32;
               dbits : Integer; dr, dg, db, da : int32);

Var
  srcformat, destformat : TPTCFormat;
  src, dest : TPTCSurface;
  F : File;

Begin
  Writeln(sbits, ' ', sr, ' ', sg, ' ', sb, ' ', dbits, ' ', dr, ' ', dg, ' ', db, ' ', da);
  srcformat := TPTCFormat.Create(sbits, sr, sg, sb);
  destformat := TPTCFormat.Create(dbits, dr, dg, db, da);
  src := TPTCSurface.Create(320, 200, srcformat);
  dest := TPTCSurface.Create(destXSize, destYSize, destformat);
  
  generic(image, src);
  src.copy(dest);
{  generic(src, dest);}
  generic(dest, surface);
  
  src.Destroy;
  dest.Destroy;
  srcformat.Destroy;
  destformat.Destroy;
  
  Inc(TestNum);
  ASSign(F, 'test' + IntToStr(TestNum) + '.raw');
  Rewrite(F, 1);
  BlockWrite(F, surface.lock^, surface.height * surface.pitch);
  surface.unlock;
  Close(F);
End;

Procedure test(sbits : Integer; sr, sg, sb : int32;
               dbits : Integer; dr, dg, db : int32);

Begin
  test(sbits, sr, sg, sb, dbits, dr, dg, db, 0);
End;

Procedure load(surface : TPTCSurface; filename : String);

Var
  F : File;
  width, height : Integer;
  pixels : PByte;
  y : Integer;
  tmp : TPTCFormat;
  tmp2 : TPTCPalette;

Begin
  ASSign(F, filename);
  Reset(F, 1);
  Seek(F, 18);
  width := surface.width;
  height := surface.height;
  pixels := surface.lock;
  For y := height - 1 DownTo 0 Do
    BlockRead(F, pixels[width * y * 3], width * 3);
  surface.unlock;
End;

Begin
  TestNum := 0;
  Try
    {$IFDEF FPC_LITTLE_ENDIAN}
    format := TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF);
    {$ELSE FPC_LITTLE_ENDIAN}
    format := TPTCFormat.Create(24, $000000FF, $0000FF00, $00FF0000);
    {$ENDIF FPC_LITTLE_ENDIAN}
    surface := TPTCSurface.Create(destXSize, destYSize, format);
    
    image := TPTCSurface.Create(320, 200, format);
    load(image, '../examples/image.tga');
    format.Free;

    
    Writeln('testing equal converters');
    {test equal converters}
    test(32, $00FF0000, $0000FF00, $000000FF, 32, $00FF0000, $0000FF00, $000000FF);
    test(24, $FF0000, $00FF00, $0000FF, 24, $FF0000, $00FF00, $0000FF);
    test(16, $F800, $07E0, $001F, 16, $F800,$07E0, $001F);
    test( 8, $E0, $1C, $03, 8, $E0, $1C, $03);

    Writeln('testing generic converters');
    {test generic}
    test(32, $FF000000, $000000FF, $000FF000, 32, $000FF000, $0FF00000, $000000FF);
    test(32, $FF000000, $000000FF, $000FF000, 24, $00FF00, $FF0000, $000000FF);
    test(32, $FF000000, $000000FF, $000FF000, 16, $F000, $0F00, $00F0);
    test(32, $FF000000, $000000FF, $000FF000, 8, $0C, $03, $F0);
    test(24, $FF0000, $0000FF, $00FF00, 32, $000FF000, $0FF00000, $000000FF);
    test(24, $FF0000, $0000FF, $00FF00, 24, $00FF00, $FF0000, $000000FF);
    test(24, $FF0000, $0000FF, $00FF00, 16, $F000, $0F00, $00F0);
    test(24, $FF0000, $0000FF, $00FF00, 8, $0C, $03, $F0);
    test(16, $001F, $F800, $07E0, 32, $000FF000, $0FF00000, $000000FF);
    test(16, $001F, $F800, $07E0, 24, $00FF00, $FF0000, $000000FF);
    test(16, $001F, $F800, $07E0, 16, $F000, $0F00, $00F0);
    test(16, $001F, $F800, $07E0, 8, $0C, $03, $F0);
//    test(8, $03, $E0, $1C, 32, $000FF000, $0FF00000, $000000FF); {unsupported}
//    test(8, $03, $E0, $1C, 24, $00FF00, $FF0000, $000000FF); {unsupported}
//    test(8, $03, $E0, $1C, 16, $F000, $0F00, $00F0); {unsupported}
//    test(8, $03, $E0, $1C, 8, $0C, $03, $F0); {unsupported}

    Writeln('testing specialized converters');
    {From 32 bit RGB 888}
    test(32,$ff0000,$ff00,$ff,16,$f800,$7e0,$1f);          {16RGB565 }
    test(32,$ff0000,$ff00,$ff, 8,$e0,$1c,$3);              { 8RGB332 }
    test(32,$ff0000,$ff00,$ff,16,$7c00,$3e0,$1f);          { 16RGB555 }
    test(32,$ff0000,$ff00,$ff,24,$ff0000,$ff00,$ff);       { 24RGB888 }
    test(32,$ff0000,$ff00,$ff,32,$ff,$ff00,$ff0000);       { 32BGR888 }
    test(32,$ff0000,$ff00,$ff,16,$1f,$7e0,$f800);          { 16BGR565 }
    test(32,$ff0000,$ff00,$ff,16,$1f,$3e0,$7c00);          { 16BGR555 }
    test(32,$ff0000,$ff00,$ff,32,$ff000000,$ff0000,$ff00,$ff); { 32RGBA888 }
    test(32,$ff0000,$ff00,$ff,32,$ff00,$ff0000,$ff000000,$ff); { 32BGRA888 }
    test(32,$ff0000,$ff00,$ff,24,$ff,$ff00,$ff0000);       { 24BGR888 }
    {From 24 bit RGB 888}
    test(24,$ff0000,$ff00,$ff,32,$ff0000,$ff00,$ff);       { 32RGB888 }
    test(24,$ff0000,$ff00,$ff,16,$f800,$7e0,$1f);          { 16RGB565 }
    test(24,$ff0000,$ff00,$ff, 8,$e0,$1c,$3);              { 8RGB332 }
    test(24,$ff0000,$ff00,$ff,16,$7c00,$3e0,$1f);          { 16RGB555 }
    test(24,$ff0000,$ff00,$ff,32,$ff,$ff00,$ff0000);       { 32BGR888 }
    test(24,$ff0000,$ff00,$ff,16,$1f,$7e0,$f800);          { 16BGR565 }
    test(24,$ff0000,$ff00,$ff,16,$1f,$3e0,$7c00);          { 16BGR555 }
    test(24,$ff0000,$ff00,$ff,32,$ff000000,$ff0000,$ff00,$ff); { 32RGBA888 }
    test(24,$ff0000,$ff00,$ff,32,$ff00,$ff0000,$ff000000,$ff); { 32BGRA888 }
    test(24,$ff0000,$ff00,$ff,24,$ff,$ff00,$ff0000);       { 24BGR888 }
    {From 16 bit RGB 565}
    test(16,$f800,$7e0,$1f,32,$ff0000,$ff00,$ff);          { 32RGB888 }
    test(16,$f800,$7e0,$1f, 8,$e0,$1c,$3);                 { 8RGB332 }
    test(16,$f800,$7e0,$1f,16,$7c00,$3e0,$1f);             { 16RGB555 }
    test(16,$f800,$7e0,$1f,24,$ff0000,$ff00,$ff);          { 24RGB888 }
    test(16,$f800,$7e0,$1f,32,$ff,$ff00,$ff0000);          { 32BGR888 }
    test(16,$f800,$7e0,$1f,16,$1f,$7e0,$f800);             { 16BGR565 }
    test(16,$f800,$7e0,$1f,16,$1f,$3e0,$7c00);             { 16BGR555 }
    test(16,$f800,$7e0,$1f,32,$ff000000,$ff0000,$ff00,$ff);    { 32RGBA888 }
    test(16,$f800,$7e0,$1f,32,$ff00,$ff0000,$ff000000,$ff);    { 32BGRA888 }
    test(16,$f800,$7e0,$1f,24,$ff,$ff00,$ff0000);          { 24BGR888 }
    {From 32 bit muhmu}
    test(32,$ff00000,$3fc00,$ff,32,$ff0000,$ff00,$ff);     { 32RGB888 }
    test(32,$ff00000,$3fc00,$ff,16,$f800,$7e0,$1f);        { 16RGB565 }
    test(32,$ff00000,$3fc00,$ff, 8,$e0,$1c,$3);            { 8RGB332 }
    test(32,$ff00000,$3fc00,$ff,16,$7c00,$3e0,$1f);        { 16RGB555 }
    test(32,$ff00000,$3fc00,$ff,24,$ff0000,$ff00,$ff);     { 24RGB888 }
    test(32,$ff00000,$3fc00,$ff,32,$ff,$ff00,$ff0000);     { 32BGR888 }
    test(32,$ff00000,$3fc00,$ff,16,$1f,$7e0,$f800);        { 16BGR565 }
    test(32,$ff00000,$3fc00,$ff,16,$1f,$3e0,$7c00);        { 16BGR555 }
    test(32,$ff00000,$3fc00,$ff,32,$ff000000,$ff0000,$ff00,$ff); { 32RGBA888 }
    test(32,$ff00000,$3fc00,$ff,32,$ff00,$ff0000,$ff000000,$ff); { 32BGRA888 }
    test(32,$ff00000,$3fc00,$ff,24,$ff,$ff00,$ff0000);     { 24BGR888 }
    
    
    surface.Destroy;
    image.Destroy;
  Except
    On error : TPTCError Do
      error.report;
  End;
End.
