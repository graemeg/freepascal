program convtest;

{$MODE objfpc}

{$I endian.inc}

uses
  SysUtils, ptc;

const
  destXSize = {480}320;
  destYSize = {300}200;

var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  image: IPTCSurface;
  surface: IPTCSurface;
  format: IPTCFormat;
=======
  image: TPTCSurface = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
>>>>>>> graemeg/cpstrnew
=======
  image: TPTCSurface = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
>>>>>>> graemeg/cpstrnew
=======
  image: TPTCSurface = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
>>>>>>> graemeg/cpstrnew
=======
  image: TPTCSurface = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
>>>>>>> origin/cpstrnew
  TestNum: Integer;

function fb(q: Uint32): Integer;
begin
  fb := 0;
  while (q and 1) = 0 do
  begin
    Inc(fb);
    q := q shr 1;
  end;
end;

function nb(q: Uint32): Integer;
begin
  nb := 0;
  while q <> 0 do
  begin
    Inc(nb);
    q := q and (q - 1);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure generic(src, dest: IPTCSurface);
=======
procedure generic(src, dest: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure generic(src, dest: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure generic(src, dest: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure generic(src, dest: TPTCSurface);
>>>>>>> origin/cpstrnew
var
  X, Y: Integer;
  XSize, YSize: Integer;
  r, g, b: Uint32;
  pix: Uint32;
  Psrc, Pdest: PUint8;
  srcbits: Integer;
  Srmask, Sgmask, Sbmask: Uint32;
  Srmasknb, Sgmasknb, Sbmasknb: Integer;
  Srmaskfb, Sgmaskfb, Sbmaskfb: Integer;
  destbits: Integer;
  Drmask, Dgmask, Dbmask: Uint32;
  Drmasknb, Dgmasknb, Dbmasknb: Integer;
  Drmaskfb, Dgmaskfb, Dbmaskfb: Integer;
begin
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
  try
    Pdest := dest.lock;
    try
      for Y := 0 to YSize - 1 do
        for X := 0 to XSize - 1 do
        begin
          case srcbits of
            32: begin
              pix := (PUint32(Psrc))^;
              Inc(Psrc, 4);
            end;
            24: begin
              {$IFDEF FPC_LITTLE_ENDIAN}
                pix := (Psrc^) or ((Psrc + 1)^ shl 8) or ((Psrc + 2)^ shl 16);
              {$ELSE FPC_LITTLE_ENDIAN}
                pix := (Psrc^ shl 16) or ((Psrc + 1)^ shl 8) or ((Psrc + 2)^);
              {$ENDIF FPC_LITTLE_ENDIAN}
              Inc(Psrc, 3);
            end;
            16: begin
              pix := (PUint16(Psrc))^;
              Inc(Psrc, 2);
            end;
            8: begin
              pix := Psrc^;
              Inc(Psrc);
            end;
          end;

          r := pix and Srmask;
          g := pix and Sgmask;
          b := pix and Sbmask;
          r := r shr Srmaskfb;
          g := g shr Sgmaskfb;
          b := b shr Sbmaskfb;

          if (Drmasknb - Srmasknb) >= 0 then
            r := r shl (Drmasknb - Srmasknb)
          else
            r := r shr (Srmasknb - Drmasknb);
          if (Dgmasknb - Sgmasknb) >= 0 then
            g := g shl (Dgmasknb - Sgmasknb)
          else
            g := g shr (Sgmasknb - Dgmasknb);
          if (Dbmasknb - Sbmasknb) >= 0 then
            b := b shl (Dbmasknb - Sbmasknb)
          else
            b := b shr (Sbmasknb - Dbmasknb);

          r := r shl Drmaskfb;
          g := g shl Dgmaskfb;
          b := b shl Dbmaskfb;
          pix := r or g or b;

          case destbits of
            32: begin
              (PUint32(Pdest))^ := pix;
              Inc(Pdest, 4);
            end;
            24: begin
              {$IFDEF FPC_LITTLE_ENDIAN}
                Pdest^ := pix and $FF;
                (Pdest + 1)^ := (pix shr 8) and $FF;
                (Pdest + 2)^ := (pix shr 16) and $FF;
              {$ELSE FPC_LITTLE_ENDIAN}
                Pdest^ := (pix shr 16) and $FF;
                (Pdest + 1)^ := (pix shr 8) and $FF;
                (Pdest + 2)^ := pix and $FF;
              {$ENDIF FPC_LITTLE_ENDIAN}
              Inc(Pdest, 3);
            end;
            16: begin
              (PUint16(Pdest))^ := pix;
              Inc(Pdest, 2);
            end;
            8: begin
              Pdest^ := pix;
              Inc(Pdest);
            end;
          end;
        end;
    finally
      dest.unlock;
    end;
  finally
    src.unlock;
  end;
end;

procedure test(sbits: Integer; sr, sg, sb: Uint32;
               dbits: Integer; dr, dg, db: Uint32; da: Uint32 = 0;
               dithering: Boolean = False);
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  srcformat, destformat: IPTCFormat;
  src, dest: IPTCSurface;
=======
  srcformat, destformat: TPTCFormat;
  src, dest: TPTCSurface;
>>>>>>> graemeg/cpstrnew
=======
  srcformat, destformat: TPTCFormat;
  src, dest: TPTCSurface;
>>>>>>> graemeg/cpstrnew
=======
  srcformat, destformat: TPTCFormat;
  src, dest: TPTCSurface;
>>>>>>> graemeg/cpstrnew
=======
  srcformat, destformat: TPTCFormat;
  src, dest: TPTCSurface;
>>>>>>> origin/cpstrnew
  pixels: Pointer;
  F: File;
begin
  Writeln(sbits, ' ', sr, ' ', sg, ' ', sb, ' ', dbits, ' ', dr, ' ', dg, ' ', db, ' ', da);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  srcformat := TPTCFormatFactory.CreateNew(sbits, sr, sg, sb);
  destformat := TPTCFormatFactory.CreateNew(dbits, dr, dg, db, da);
  src := TPTCSurfaceFactory.CreateNew(320, 200, srcformat);
  dest := TPTCSurfaceFactory.CreateNew(destXSize, destYSize, destformat);

  if dithering then
    dest.Option('attempt dithering');

  generic(image, src);
  src.copy(dest);
{    generic(src, dest);}
  generic(dest, surface);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  try
    srcformat := TPTCFormat.Create(sbits, sr, sg, sb);
    destformat := TPTCFormat.Create(dbits, dr, dg, db, da);
    src := TPTCSurface.Create(320, 200, srcformat);
    dest := TPTCSurface.Create(destXSize, destYSize, destformat);

    if dithering then
      dest.Option('attempt dithering');

    generic(image, src);
    src.copy(dest);
{    generic(src, dest);}
    generic(dest, surface);
  finally
    src.Free;
    dest.Free;
    srcformat.Free;
    destformat.Free;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  Inc(TestNum);
  AssignFile(F, 'test' + IntToStr(TestNum) + '.raw');
  Rewrite(F, 1);
  try
    pixels := surface.lock;
    try
      BlockWrite(F, pixels^, surface.height * surface.pitch);
    finally
      surface.unlock;
    end;
  finally
    CloseFile(F);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure load(surface: IPTCSurface; filename: String);
=======
procedure load(surface: TPTCSurface; filename: String);
>>>>>>> graemeg/cpstrnew
=======
procedure load(surface: TPTCSurface; filename: String);
>>>>>>> graemeg/cpstrnew
=======
procedure load(surface: TPTCSurface; filename: String);
>>>>>>> graemeg/cpstrnew
=======
procedure load(surface: TPTCSurface; filename: String);
>>>>>>> origin/cpstrnew
var
  F: File;
  width, height: Integer;
  pixels: PByte;
  y: Integer;
begin
  AssignFile(F, filename);
  Reset(F, 1);
  try
    Seek(F, 18);
    width := surface.width;
    height := surface.height;
    pixels := surface.lock;
    try
      for y := height - 1 downto 0 do
        BlockRead(F, pixels[width * y * 3], width * 3);
    finally
      surface.unlock;
    end;
  finally
    CloseFile(F);
  end;
end;

begin
  TestNum := 0;
  try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    {$IFDEF FPC_LITTLE_ENDIAN}
    format := TPTCFormatFactory.CreateNew(24, $00FF0000, $0000FF00, $000000FF);
    {$ELSE FPC_LITTLE_ENDIAN}
    format := TPTCFormatFactory.CreateNew(24, $000000FF, $0000FF00, $00FF0000);
    {$ENDIF FPC_LITTLE_ENDIAN}
    surface := TPTCSurfaceFactory.CreateNew(destXSize, destYSize, format);

    image := TPTCSurfaceFactory.CreateNew(320, 200, format);
    load(image, '../examples/image.tga');


    Writeln('testing equal converters');
    {test equal converters}
    test(32, $00FF0000, $0000FF00, $000000FF, 32, $00FF0000, $0000FF00, $000000FF); { 1 }
    test(24, $FF0000, $00FF00, $0000FF, 24, $FF0000, $00FF00, $0000FF);             { 2 }
    test(16, $F800, $07E0, $001F, 16, $F800,$07E0, $001F);                          { 3 }
    test( 8, $E0, $1C, $03, 8, $E0, $1C, $03);                                      { 4 }

    Writeln('testing generic converters');
    {test generic}
    test(32, $FF000000, $000000FF, $000FF000, 32, $000FF000, $0FF00000, $000000FF); { 5 }
    test(32, $FF000000, $000000FF, $000FF000, 24, $00FF00, $FF0000, $000000FF);     { 6 }
    test(32, $FF000000, $000000FF, $000FF000, 16, $F000, $0F00, $00F0);             { 7 }
    test(32, $FF000000, $000000FF, $000FF000, 8, $0C, $03, $F0);                    { 8 }
    test(24, $FF0000, $0000FF, $00FF00, 32, $000FF000, $0FF00000, $000000FF);       { 9 }
    test(24, $FF0000, $0000FF, $00FF00, 24, $00FF00, $FF0000, $000000FF);           { 10 }
    test(24, $FF0000, $0000FF, $00FF00, 16, $F000, $0F00, $00F0);                   { 11 }
    test(24, $FF0000, $0000FF, $00FF00, 8, $0C, $03, $F0);                          { 12 }
    test(16, $001F, $F800, $07E0, 32, $000FF000, $0FF00000, $000000FF);             { 13 }
    test(16, $001F, $F800, $07E0, 24, $00FF00, $FF0000, $000000FF);                 { 14 }
    test(16, $001F, $F800, $07E0, 16, $F000, $0F00, $00F0);                         { 15 }
    test(16, $001F, $F800, $07E0, 8, $0C, $03, $F0);                                { 16 }
//    test(8, $03, $E0, $1C, 32, $000FF000, $0FF00000, $000000FF); {unsupported}
//    test(8, $03, $E0, $1C, 24, $00FF00, $FF0000, $000000FF); {unsupported}
//    test(8, $03, $E0, $1C, 16, $F000, $0F00, $00F0); {unsupported}
//    test(8, $03, $E0, $1C, 8, $0C, $03, $F0); {unsupported}

    Writeln('testing specialized converters');
    {From 32 bit RGB 888}
    test(32,$ff0000,$ff00,$ff,16,$f800,$7e0,$1f);                { 16RGB565  }      { 17 }
    test(32,$ff0000,$ff00,$ff, 8,$e0,$1c,$3);                    { 8RGB332   }      { 18 }
    test(32,$ff0000,$ff00,$ff,16,$7c00,$3e0,$1f);                { 16RGB555  }      { 19 }
    test(32,$ff0000,$ff00,$ff,24,$ff0000,$ff00,$ff);             { 24RGB888  }      { 20 }
    test(32,$ff0000,$ff00,$ff,32,$ff,$ff00,$ff0000);             { 32BGR888  }      { 21 }
    test(32,$ff0000,$ff00,$ff,16,$1f,$7e0,$f800);                { 16BGR565  }      { 22 }
    test(32,$ff0000,$ff00,$ff,16,$1f,$3e0,$7c00);                { 16BGR555  }      { 23 }
    test(32,$ff0000,$ff00,$ff,32,$ff000000,$ff0000,$ff00,$ff);   { 32RGBA888 }      { 24 }
    test(32,$ff0000,$ff00,$ff,32,$ff00,$ff0000,$ff000000,$ff);   { 32BGRA888 }      { 25 }
    test(32,$ff0000,$ff00,$ff,24,$ff,$ff00,$ff0000);             { 24BGR888  }      { 26 }
    {From 24 bit RGB 888}
    test(24,$ff0000,$ff00,$ff,32,$ff0000,$ff00,$ff);             { 32RGB888  }      { 27 }
    test(24,$ff0000,$ff00,$ff,16,$f800,$7e0,$1f);                { 16RGB565  }      { 28 }
    test(24,$ff0000,$ff00,$ff, 8,$e0,$1c,$3);                    { 8RGB332   }      { 29 }
    test(24,$ff0000,$ff00,$ff,16,$7c00,$3e0,$1f);                { 16RGB555  }      { 30 }
    test(24,$ff0000,$ff00,$ff,32,$ff,$ff00,$ff0000);             { 32BGR888  }      { 31 }
    test(24,$ff0000,$ff00,$ff,16,$1f,$7e0,$f800);                { 16BGR565  }      { 32 }
    test(24,$ff0000,$ff00,$ff,16,$1f,$3e0,$7c00);                { 16BGR555  }      { 33 }
    test(24,$ff0000,$ff00,$ff,32,$ff000000,$ff0000,$ff00,$ff);   { 32RGBA888 }      { 34 }
    test(24,$ff0000,$ff00,$ff,32,$ff00,$ff0000,$ff000000,$ff);   { 32BGRA888 }      { 35 }
    test(24,$ff0000,$ff00,$ff,24,$ff,$ff00,$ff0000);             { 24BGR888  }      { 36 }
    {From 16 bit RGB 565}
    test(16,$f800,$7e0,$1f,32,$ff0000,$ff00,$ff);                { 32RGB888  }      { 37 }
    test(16,$f800,$7e0,$1f, 8,$e0,$1c,$3);                       { 8RGB332   }      { 38 }
    test(16,$f800,$7e0,$1f,16,$7c00,$3e0,$1f);                   { 16RGB555  }      { 39 }
    test(16,$f800,$7e0,$1f,24,$ff0000,$ff00,$ff);                { 24RGB888  }      { 40 }
    test(16,$f800,$7e0,$1f,32,$ff,$ff00,$ff0000);                { 32BGR888  }      { 41 }
    test(16,$f800,$7e0,$1f,16,$1f,$7e0,$f800);                   { 16BGR565  }      { 42 }
    test(16,$f800,$7e0,$1f,16,$1f,$3e0,$7c00);                   { 16BGR555  }      { 43 }
    test(16,$f800,$7e0,$1f,32,$ff000000,$ff0000,$ff00,$ff);      { 32RGBA888 }      { 44 }
    test(16,$f800,$7e0,$1f,32,$ff00,$ff0000,$ff000000,$ff);      { 32BGRA888 }      { 45 }
    test(16,$f800,$7e0,$1f,24,$ff,$ff00,$ff0000);                { 24BGR888  }      { 46 }
    {From 32 bit muhmu}
    test(32,$ff00000,$3fc00,$ff,32,$ff0000,$ff00,$ff);           { 32RGB888  }      { 47 }
    test(32,$ff00000,$3fc00,$ff,16,$f800,$7e0,$1f);              { 16RGB565  }      { 48 }
    test(32,$ff00000,$3fc00,$ff, 8,$e0,$1c,$3);                  { 8RGB332   }      { 49 }
    test(32,$ff00000,$3fc00,$ff,16,$7c00,$3e0,$1f);              { 16RGB555  }      { 50 }
    test(32,$ff00000,$3fc00,$ff,24,$ff0000,$ff00,$ff);           { 24RGB888  }      { 51 }
    test(32,$ff00000,$3fc00,$ff,32,$ff,$ff00,$ff0000);           { 32BGR888  }      { 52 }
    test(32,$ff00000,$3fc00,$ff,16,$1f,$7e0,$f800);              { 16BGR565  }      { 53 }
    test(32,$ff00000,$3fc00,$ff,16,$1f,$3e0,$7c00);              { 16BGR555  }      { 54 }
    test(32,$ff00000,$3fc00,$ff,32,$ff000000,$ff0000,$ff00,$ff); { 32RGBA888 }      { 55 }
    test(32,$ff00000,$3fc00,$ff,32,$ff00,$ff0000,$ff000000,$ff); { 32BGRA888 }      { 56 }
    test(32,$ff00000,$3fc00,$ff,24,$ff,$ff00,$ff0000);           { 24BGR888  }      { 57 }

    Writeln('testing dithering converters');
    test(32,$ff0000,$ff00,$ff,16,$f800,$7e0,$1f, 0, True);       { 16RGB565  }      { 58 }
    test(32,$ff0000,$ff00,$ff, 8,$e0,$1c,$3, 0 , True);          { 8RGB332   }      { 59 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    try
      {$IFDEF FPC_LITTLE_ENDIAN}
      format := TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF);
      {$ELSE FPC_LITTLE_ENDIAN}
      format := TPTCFormat.Create(24, $000000FF, $0000FF00, $00FF0000);
      {$ENDIF FPC_LITTLE_ENDIAN}
      surface := TPTCSurface.Create(destXSize, destYSize, format);

      image := TPTCSurface.Create(320, 200, format);
      load(image, '../examples/image.tga');


      Writeln('testing equal converters');
      {test equal converters}
      test(32, $00FF0000, $0000FF00, $000000FF, 32, $00FF0000, $0000FF00, $000000FF); { 1 }
      test(24, $FF0000, $00FF00, $0000FF, 24, $FF0000, $00FF00, $0000FF);             { 2 }
      test(16, $F800, $07E0, $001F, 16, $F800,$07E0, $001F);                          { 3 }
      test( 8, $E0, $1C, $03, 8, $E0, $1C, $03);                                      { 4 }

      Writeln('testing generic converters');
      {test generic}
      test(32, $FF000000, $000000FF, $000FF000, 32, $000FF000, $0FF00000, $000000FF); { 5 }
      test(32, $FF000000, $000000FF, $000FF000, 24, $00FF00, $FF0000, $000000FF);     { 6 }
      test(32, $FF000000, $000000FF, $000FF000, 16, $F000, $0F00, $00F0);             { 7 }
      test(32, $FF000000, $000000FF, $000FF000, 8, $0C, $03, $F0);                    { 8 }
      test(24, $FF0000, $0000FF, $00FF00, 32, $000FF000, $0FF00000, $000000FF);       { 9 }
      test(24, $FF0000, $0000FF, $00FF00, 24, $00FF00, $FF0000, $000000FF);           { 10 }
      test(24, $FF0000, $0000FF, $00FF00, 16, $F000, $0F00, $00F0);                   { 11 }
      test(24, $FF0000, $0000FF, $00FF00, 8, $0C, $03, $F0);                          { 12 }
      test(16, $001F, $F800, $07E0, 32, $000FF000, $0FF00000, $000000FF);             { 13 }
      test(16, $001F, $F800, $07E0, 24, $00FF00, $FF0000, $000000FF);                 { 14 }
      test(16, $001F, $F800, $07E0, 16, $F000, $0F00, $00F0);                         { 15 }
      test(16, $001F, $F800, $07E0, 8, $0C, $03, $F0);                                { 16 }
//      test(8, $03, $E0, $1C, 32, $000FF000, $0FF00000, $000000FF); {unsupported}
//      test(8, $03, $E0, $1C, 24, $00FF00, $FF0000, $000000FF); {unsupported}
//      test(8, $03, $E0, $1C, 16, $F000, $0F00, $00F0); {unsupported}
//      test(8, $03, $E0, $1C, 8, $0C, $03, $F0); {unsupported}

      Writeln('testing specialized converters');
      {From 32 bit RGB 888}
      test(32,$ff0000,$ff00,$ff,16,$f800,$7e0,$1f);                { 16RGB565  }      { 17 }
      test(32,$ff0000,$ff00,$ff, 8,$e0,$1c,$3);                    { 8RGB332   }      { 18 }
      test(32,$ff0000,$ff00,$ff,16,$7c00,$3e0,$1f);                { 16RGB555  }      { 19 }
      test(32,$ff0000,$ff00,$ff,24,$ff0000,$ff00,$ff);             { 24RGB888  }      { 20 }
      test(32,$ff0000,$ff00,$ff,32,$ff,$ff00,$ff0000);             { 32BGR888  }      { 21 }
      test(32,$ff0000,$ff00,$ff,16,$1f,$7e0,$f800);                { 16BGR565  }      { 22 }
      test(32,$ff0000,$ff00,$ff,16,$1f,$3e0,$7c00);                { 16BGR555  }      { 23 }
      test(32,$ff0000,$ff00,$ff,32,$ff000000,$ff0000,$ff00,$ff);   { 32RGBA888 }      { 24 }
      test(32,$ff0000,$ff00,$ff,32,$ff00,$ff0000,$ff000000,$ff);   { 32BGRA888 }      { 25 }
      test(32,$ff0000,$ff00,$ff,24,$ff,$ff00,$ff0000);             { 24BGR888  }      { 26 }
      {From 24 bit RGB 888}
      test(24,$ff0000,$ff00,$ff,32,$ff0000,$ff00,$ff);             { 32RGB888  }      { 27 }
      test(24,$ff0000,$ff00,$ff,16,$f800,$7e0,$1f);                { 16RGB565  }      { 28 }
      test(24,$ff0000,$ff00,$ff, 8,$e0,$1c,$3);                    { 8RGB332   }      { 29 }
      test(24,$ff0000,$ff00,$ff,16,$7c00,$3e0,$1f);                { 16RGB555  }      { 30 }
      test(24,$ff0000,$ff00,$ff,32,$ff,$ff00,$ff0000);             { 32BGR888  }      { 31 }
      test(24,$ff0000,$ff00,$ff,16,$1f,$7e0,$f800);                { 16BGR565  }      { 32 }
      test(24,$ff0000,$ff00,$ff,16,$1f,$3e0,$7c00);                { 16BGR555  }      { 33 }
      test(24,$ff0000,$ff00,$ff,32,$ff000000,$ff0000,$ff00,$ff);   { 32RGBA888 }      { 34 }
      test(24,$ff0000,$ff00,$ff,32,$ff00,$ff0000,$ff000000,$ff);   { 32BGRA888 }      { 35 }
      test(24,$ff0000,$ff00,$ff,24,$ff,$ff00,$ff0000);             { 24BGR888  }      { 36 }
      {From 16 bit RGB 565}
      test(16,$f800,$7e0,$1f,32,$ff0000,$ff00,$ff);                { 32RGB888  }      { 37 }
      test(16,$f800,$7e0,$1f, 8,$e0,$1c,$3);                       { 8RGB332   }      { 38 }
      test(16,$f800,$7e0,$1f,16,$7c00,$3e0,$1f);                   { 16RGB555  }      { 39 }
      test(16,$f800,$7e0,$1f,24,$ff0000,$ff00,$ff);                { 24RGB888  }      { 40 }
      test(16,$f800,$7e0,$1f,32,$ff,$ff00,$ff0000);                { 32BGR888  }      { 41 }
      test(16,$f800,$7e0,$1f,16,$1f,$7e0,$f800);                   { 16BGR565  }      { 42 }
      test(16,$f800,$7e0,$1f,16,$1f,$3e0,$7c00);                   { 16BGR555  }      { 43 }
      test(16,$f800,$7e0,$1f,32,$ff000000,$ff0000,$ff00,$ff);      { 32RGBA888 }      { 44 }
      test(16,$f800,$7e0,$1f,32,$ff00,$ff0000,$ff000000,$ff);      { 32BGRA888 }      { 45 }
      test(16,$f800,$7e0,$1f,24,$ff,$ff00,$ff0000);                { 24BGR888  }      { 46 }
      {From 32 bit muhmu}
      test(32,$ff00000,$3fc00,$ff,32,$ff0000,$ff00,$ff);           { 32RGB888  }      { 47 }
      test(32,$ff00000,$3fc00,$ff,16,$f800,$7e0,$1f);              { 16RGB565  }      { 48 }
      test(32,$ff00000,$3fc00,$ff, 8,$e0,$1c,$3);                  { 8RGB332   }      { 49 }
      test(32,$ff00000,$3fc00,$ff,16,$7c00,$3e0,$1f);              { 16RGB555  }      { 50 }
      test(32,$ff00000,$3fc00,$ff,24,$ff0000,$ff00,$ff);           { 24RGB888  }      { 51 }
      test(32,$ff00000,$3fc00,$ff,32,$ff,$ff00,$ff0000);           { 32BGR888  }      { 52 }
      test(32,$ff00000,$3fc00,$ff,16,$1f,$7e0,$f800);              { 16BGR565  }      { 53 }
      test(32,$ff00000,$3fc00,$ff,16,$1f,$3e0,$7c00);              { 16BGR555  }      { 54 }
      test(32,$ff00000,$3fc00,$ff,32,$ff000000,$ff0000,$ff00,$ff); { 32RGBA888 }      { 55 }
      test(32,$ff00000,$3fc00,$ff,32,$ff00,$ff0000,$ff000000,$ff); { 32BGRA888 }      { 56 }
      test(32,$ff00000,$3fc00,$ff,24,$ff,$ff00,$ff0000);           { 24BGR888  }      { 57 }

      Writeln('testing dithering converters');
      test(32,$ff0000,$ff00,$ff,16,$f800,$7e0,$1f, 0, True);       { 16RGB565  }      { 58 }
      test(32,$ff0000,$ff00,$ff, 8,$e0,$1c,$3, 0 , True);          { 8RGB332   }      { 59 }
    finally
      surface.Free;
      image.Free;
      format.Free;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  except
    on error: TPTCError do
      error.report;
  end;
end.
