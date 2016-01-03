{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2011 Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

{$MACRO ON}

{$DEFINE DEFAULT_WIDTH:=320}
{$DEFINE DEFAULT_HEIGHT:=200}
{$DEFINE DEFAULT_FORMAT:=TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF)}
{ $DEFINE DEFAULT_FORMAT:=TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF)}

procedure VESALogProcedure(const S: string);
begin
  LOG(s);
end;

constructor TVESAConsole.Create;
begin
  inherited Create;
  
  FTryLFB := true;
  FTryWindowed := true;
  
  vesa.LogProcedure := @VESALogProcedure;

  FOpen := False;
  FLocked := False;
  FTitle := '';
  FInformation := '';
  FDefaultWidth := DEFAULT_WIDTH;
  FDefaultHeight := DEFAULT_HEIGHT;
  FDefaultFormat := DEFAULT_FORMAT;

  FPrimary := nil;
  InitVESA;
  if not VBEPresent then
    raise TPTCError.Create('the system does not support the VESA BIOS extensions');

  UpdateModeList;

  FClip := TPTCArea.Create;
  FArea := TPTCArea.Create;
  FCopy := TPTCCopy.Create;
  FPalette := TPTCPalette.Create;
  Configure('ptcpas.cfg');
end;

destructor TVESAConsole.Destroy;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Close;
  
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  I: Integer;
begin
  Close;
  
  for I := Low(FModes) to High(FModes) do
    FreeAndNil(FModes[I]);
  SetLength(FModes, 0);
  SetLength(FModesN, 0);
  
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
  FKeyboard.Free;
  FMouse.Free;
  FEventQueue.Free;
  FCopy.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FDefaultFormat.Free;
  FPalette.Free;
  FArea.Free;
  FClip.Free;
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
  inherited Destroy;
end;

procedure TVESAConsole.UpdateModeList;
var
  I, J: Integer;
  r, g, b, a: DWord;
  tmpbpp: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  tmp: TPTCFormat;
begin
  for I := Low(FModes) to High(FModes) do
    FreeAndNil(FModes[I]);
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
  SetLength(FModes, 0);
  SetLength(FModesN, 0);

(*  FModesLast := -1;
  for I := Low(VBEModes) to High(VBEModes) do
    with VBEModes[I] do
      if (MemoryModel = vmmmDirectColor) and
         ((BitsPerPixel = 8) or
          (BitsPerPixel = 15) or
          (BitsPerPixel = 16) or
          (BitsPerPixel = 24) or
          (BitsPerPixel = 32)) then
        Inc(FModesLast)
      else
        if (MemoryModel = vmmmPackedPixel) and (BitsPerPixel = 8) then
          Inc(FModesLast, 2);
  SetLength(FModes, FModesLast + 2);
  SetLength(FModesN, FModesLast + 1);
//  Writeln(FModesLast, ' ', NrOfModes);
  FModes[FModesLast + 1] := TPTCMode.Create; {mark end of list!} *)
  J := -1;
  for I := Low(VBEModes) to High(VBEModes) do
    with VBEModes[I] do
      if (FTryWindowed and SupportsWindowed) or (FTryLFB and SupportsLFB) then
        if (MemoryModel = vmmmDirectColor) and
           ((BitsPerPixel = 8) or
            (BitsPerPixel = 15) or
            (BitsPerPixel = 16) or
            (BitsPerPixel = 24) or
            (BitsPerPixel = 32)) then
        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          if FTryWindowed and SupportsWindowed then
          begin
=======
	  if FTryWindowed and SupportsWindowed then
	  begin
>>>>>>> graemeg/cpstrnew
=======
	  if FTryWindowed and SupportsWindowed then
	  begin
>>>>>>> graemeg/cpstrnew
=======
	  if FTryWindowed and SupportsWindowed then
	  begin
>>>>>>> graemeg/cpstrnew
=======
	  if FTryWindowed and SupportsWindowed then
	  begin
>>>>>>> origin/cpstrnew
            Inc(J);
            r := MakeMask(WindowedRedMaskSize, WindowedRedFieldPosition);
            g := MakeMask(WindowedGreenMaskSize, WindowedGreenFieldPosition);
            b := MakeMask(WindowedBlueMaskSize, WindowedBlueFieldPosition);
            {a := MakeMask(RsvdMaskSize, RsvdFieldPosition);}
            a := 0;
            if BitsPerPixel = 15 then
              tmpbpp := 16
            else
              tmpbpp := BitsPerPixel;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            SetLength(FModes, J + 1);
            SetLength(FModesN, J + 1);
            FModes[J] := TPTCMode.Create(XResolution, YResolution, TPTCFormat.Create(tmpbpp, r, g, b, a));
            FModesN[J].Index := I;
            FModesN[J].SupportsWindowed := true;
          end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            tmp := TPTCFormat.Create(tmpbpp, r, g, b, a);
            try
	      SetLength(FModes, J + 1);
	      SetLength(FModesN, J + 1);
              FModes[J] := TPTCMode.Create(XResolution, YResolution, tmp);
              FModesN[J].Index := I;
	      FModesN[J].SupportsWindowed := true;
            finally
              tmp.Free;
            end;
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
	  
	  if FTryLFB and SupportsLFB then
	  begin
	    if (FTryWindowed and SupportsWindowed) and
	       (WindowedRedMaskSize        = LFBRedMaskSize) and
	       (WindowedRedFieldPosition   = LFBRedFieldPosition) and
	       (WindowedGreenMaskSize      = LFBGreenMaskSize) and
	       (WindowedGreenFieldPosition = LFBGreenFieldPosition) and
	       (WindowedBlueMaskSize       = LFBBlueMaskSize) and
	       (WindowedBlueFieldPosition  = LFBBlueFieldPosition) then
	    begin
	      { the same as the windowed mode => do not add a new mode, just
	        set the LFB flag of the last one }
	      FModesN[J].SupportsLFB := true;
	    end
	    else
	    begin
              Inc(J);
              r := MakeMask(LFBRedMaskSize, LFBRedFieldPosition);
              g := MakeMask(LFBGreenMaskSize, LFBGreenFieldPosition);
              b := MakeMask(LFBBlueMaskSize, LFBBlueFieldPosition);
              {a := MakeMask(RsvdMaskSize, RsvdFieldPosition);}
              a := 0;
              if BitsPerPixel = 15 then
                tmpbpp := 16
              else
                tmpbpp := BitsPerPixel;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              SetLength(FModes, J + 1);
              SetLength(FModesN, J + 1);
              FModes[J] := TPTCMode.Create(XResolution, YResolution, TPTCFormat.Create(tmpbpp, r, g, b, a));
              FModesN[J].Index := I;
              FModesN[J].SupportsLFB := true;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              tmp := TPTCFormat.Create(tmpbpp, r, g, b, a);
              try
	        SetLength(FModes, J + 1);
	        SetLength(FModesN, J + 1);
                FModes[J] := TPTCMode.Create(XResolution, YResolution, tmp);
                FModesN[J].Index := I;
  	        FModesN[J].SupportsLFB := true;
              finally
                tmp.Free;
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
	    end;
	  end;
{          Inc(FModesLast)}
        end
        else
          if (MemoryModel = vmmmPackedPixel) and (BitsPerPixel = 8) then
          begin
            Inc(J);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            SetLength(FModes, J + 1);
            SetLength(FModesN, J + 1);
            FModes[J] := TPTCMode.Create(XResolution, YResolution, TPTCFormat.Create(8));
            FModesN[J].Index := I;
            FModesN[J].SupportsWindowed := FTryWindowed and SupportsWindowed;
            FModesN[J].SupportsLFB := FTryLFB and SupportsLFB;
            Inc(J);
			
            {RGB 332}
            SetLength(FModes, J + 1);
            SetLength(FModesN, J + 1);
            FModes[J] := TPTCMode.Create(XResolution, YResolution, TPTCFormat.Create(8, $E0, $1C, $03));
            FModesN[J].Index := I;
            FModesN[J].SupportsWindowed := FTryWindowed and SupportsWindowed;
            FModesN[J].SupportsLFB := FTryLFB and SupportsLFB;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            tmp := TPTCFormat.Create(8);
            try
              SetLength(FModes, J + 1);
              SetLength(FModesN, J + 1);
              FModes[J] := TPTCMode.Create(XResolution, YResolution, tmp);
              FModesN[J].Index := I;
	      FModesN[J].SupportsWindowed := FTryWindowed and SupportsWindowed;
	      FModesN[J].SupportsLFB := FTryLFB and SupportsLFB;
            finally
              tmp.Free;
            end;
            Inc(J);
            tmp := TPTCFormat.Create(8, $E0, $1C, $03); {RGB 332}
            try
              SetLength(FModes, J + 1);
              SetLength(FModesN, J + 1);
              FModes[J] := TPTCMode.Create(XResolution, YResolution, tmp);
              FModesN[J].Index := I;
	      FModesN[J].SupportsWindowed := FTryWindowed and SupportsWindowed;
	      FModesN[J].SupportsLFB := FTryLFB and SupportsLFB;
            finally
              tmp.Free;
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
{           Inc(FModesLast, 2);}
          end;

  FModesLast := J;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TVESAConsole.Configure(const AFileName: string);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SetLength(FModes, FModesLast + 2);
  FModes[FModesLast + 1] := TPTCMode.Create; {mark end of list!}
end;

procedure TVESAConsole.Configure(const AFileName: String);
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
var
  F: TextFile;
  S: string;
begin
  AssignFile(F, AFileName);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$push}{$I-}
  Reset(F);
  {$pop}
=======
  {$I-}
  Reset(F);
  {$I+}
>>>>>>> graemeg/cpstrnew
=======
  {$I-}
  Reset(F);
  {$I+}
>>>>>>> graemeg/cpstrnew
=======
  {$I-}
  Reset(F);
  {$I+}
>>>>>>> graemeg/cpstrnew
=======
  {$I-}
  Reset(F);
  {$I+}
>>>>>>> origin/cpstrnew
  if IOResult <> 0 then
    exit;
  while not EoF(F) do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    {$push}{$I-}
    Readln(F, S);
    {$pop}
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> origin/cpstrnew
    if IOResult <> 0 then
      Break;
    Option(S);
  end;
  CloseFile(F);
end;

procedure TVESAConsole.EnableLFB;
begin
  if FTryLFB <> true then
  begin
    FTryLFB := true;
    UpdateModeList;
  end;
end;

procedure TVESAConsole.DisableLFB;
begin
  if FTryLFB <> false then
  begin
    FTryLFB := false;
    UpdateModeList;
  end;
end;

procedure TVESAConsole.EnableWindowed;
begin
  if FTryWindowed <> true then
  begin
    FTryWindowed := true;
    UpdateModeList;
  end;
end;

procedure TVESAConsole.DisableWindowed;
begin
  if FTryWindowed <> false then
  begin
    FTryWindowed := false;
    UpdateModeList;
  end;
end;

function TVESAConsole.Option(const AOption: string): Boolean;
begin
  {...}
  if AOption = 'enable lfb' then
  begin
    EnableLFB;
    Result := true;
    exit;
  end;
  if AOption = 'disable lfb' then
  begin
    DisableLFB;
    Result := true;
    exit;
  end;
  if AOption = 'default lfb' then
  begin
    EnableLFB;
    Result := true;
    exit;
  end;
  
  if AOption = 'enable windowed framebuffer' then
  begin
    EnableWindowed;
    Result := true;
    exit;
  end;
  if AOption = 'disable windowed framebuffer' then
  begin
    DisableWindowed;
    Result := true;
    exit;
  end;
  if AOption = 'default windowed framebuffer' then
  begin
    EnableWindowed;
    Result := true;
    exit;
  end;
  
  if AOption = 'enable dpmi508h' then
  begin
    VESA.TryDPMI508h := true;
    Result := true;
    exit;
  end;
  if AOption = 'disable dpmi508h' then
  begin
    VESA.TryDPMI508h := false;
    Result := true;
    exit;
  end;
  if AOption = 'default dpmi508h' then
  begin
    VESA.TryDPMI508h := VESA.TryDPMI508hDefault;
    Result := true;
    exit;
  end;
  
  if AOption = 'enable nearptr' then
  begin
    VESA.TryNearPtr := true;
    Result := true;
    exit;
  end;
  if AOption = 'disable nearptr' then
  begin
    VESA.TryNearPtr := false;
    Result := true;
    exit;
  end;
  if AOption = 'default nearptr' then
  begin
    VESA.TryNearPtr := VESA.TryNearPtrDefault;
    Result := true;
    exit;
  end;
  
  if AOption = 'no8bitdac' then
  begin
    VESA.EightBitDACEnabled := false;
    Result := true;
    exit;
  end;
  
  if AOption = 'enable logging' then
  begin
    LOG_enabled := true;
    Result := true;
    exit;
  end;
  if AOption = 'disable logging' then
  begin
    LOG_enabled := false;
    Result := true;
    exit;
  end;

  Result := FCopy.Option(AOption);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVESAConsole.Modes: TPTCModeList;
begin
  Result := FModes;
end;

function TVESAConsole.FindBestMode(const AMode: IPTCMode): Integer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TVESAConsole.Modes: PPTCMode;
begin
  Result := @FModes[0];
end;

function TVESAConsole.FindBestMode(const AMode: TPTCMode): Integer;
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
var
  I: Integer;
  modefound, bestmodefound: Integer;
  x, y, bpp: Integer;
begin
  if not AMode.Valid then
    raise TPTCError.Create('invalid mode');

  for I := 0 to FModesLast do
    if FModes[I].Equals(AMode) then
    begin
      Result := I;
      exit;
    end;

  modefound := -1;
  bestmodefound := -1;
  if (modefound = -1) and (AMode.Format.Direct) then
  begin
    x := 100000000;
    y := x;
    bpp := -1;
    for I := 0 to FModesLast do
      if (FModes[i].Width >= AMode.Width) and
         (FModes[i].Height >= AMode.Height) and
         (FModes[i].Width <= x) and
         (FModes[i].Height <= y) and
         (((FModes[i].Format.Bits >= bpp) and
           (bpp < AMode.Format.Bits)) or
          ((FModes[i].Format.Bits < bpp) and
           (FModes[i].Format.Bits >= AMode.Format.Bits) and
           (bpp > AMode.Format.Bits))) then
      begin
        bestmodefound := I;
        x := FModes[i].Width;
        y := FModes[i].Height;
        bpp := FModes[i].Format.Bits;
      end;
{      if FModes[I].bpp >=  then
      begin
        modefound := I;
        Break;
      End;}
  end;
  if (modefound = -1) and (AMode.format.indexed) then
  begin
    x := 100000000;
    y := x;
    bpp := -1;
    for I := 0 to FModesLast do
      if (FModes[i].Width >= AMode.Width) and
         (FModes[i].Height >= AMode.Height) and
         (FModes[i].Width <= x) and
         (FModes[i].Height <= y) { and
         (((FModes[i].format.bits >= bpp) and
           (bpp < _mode.format.bits)) or
          ((FModes[i].format.bits < bpp) and
           (FModes[i].format.bits >= _mode.format.bits) and
           (bpp > _mode.format.bits)))} then
      begin
        if (FModes[i].Width <> x) or (FModes[i].Height <> y) then
          bpp := -1;
        if FModes[i].Format.Indexed or
           (FModes[i].Format.Bits > bpp) then
        begin
          bestmodefound := I;
          x := FModes[i].Width;
          y := FModes[i].Height;
          bpp := FModes[i].Format.Bits;
          if FModes[i].Format.Indexed then
            bpp := 1000;
        end;
      end;
{      if FModes[I].bpp >=  then
      begin
        modefound := I;
        Break;
      End;}
  end;

  if bestmodefound <> -1 then
    modefound := bestmodefound;

  Result := modefound;
end;

procedure TVESAConsole.Open(const ATitle: string; APages: Integer); overload;
begin
  Open(ATitle, FDefaultFormat, APages);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVESAConsole.Open(const ATitle: string; AFormat: IPTCFormat;
=======
procedure TVESAConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
>>>>>>> origin/cpstrnew
                            APages: Integer); overload;
begin
  Open(ATitle, FDefaultWidth, FDefaultHeight, AFormat, APages);
end;

procedure TVESAConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            AFormat: IPTCFormat; APages: Integer); overload;
begin
  Open(ATitle, TPTCMode.Create(AWidth, AHeight, AFormat), APages);
end;

procedure TVESAConsole.Open(const ATitle: string; AMode: IPTCMode;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            const AFormat: TPTCFormat; APages: Integer); overload;
var
  m: TPTCMode;
begin
  m := TPTCMode.Create(AWidth, AHeight, AFormat);
  try
    Open(ATitle, m, APages);
  finally
    m.Free;
  end;
end;

procedure TVESAConsole.Open(const ATitle: string; const AMode: TPTCMode;
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
                            APages: Integer); overload;

  procedure SetRGB332Palette;
  var
    I: Integer;
    plt: array [0..255] of packed record
      B, G, R, A: Byte;
    end;
  begin
    for I := 0 to 255 do
      with plt[I] do
      begin
        case I shr 5 of
          0: R := 0;
          1: R := 36;
          2: R := 73;
          3: R := 109;
          4: R := 146;
          5: R := 182;
          6: R := 219;
          7: R := 255;
        end;
        case (I shr 2) and 7 of
          0: G := 0;
          1: G := 36;
          2: G := 73;
          3: G := 109;
          4: G := 146;
          5: G := 182;
          6: G := 219;
          7: G := 255;
        end;
        case I and 3 of
          0: B := 0;
          1: B := 85;
          2: B := 170;
          3: B := 255;
        end;
        A := 0;
      end;
    SetPalette(@plt, 0, 256);
  end;

var
  ModeFound: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  tmpa: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  tmpa: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  tmpa: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  tmpa: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  ModeFound := FindBestMode(AMode);

  if ModeFound = -1 then
    raise TPTCError.Create('mode not found >:)');

  internal_close;
  FTitle := ATitle;

  FCurrentMode := ModeFound;

  if FModesN[ModeFound].SupportsLFB then
  begin
    if not SetVESAMode(FModesN[ModeFound].Index, true) then
    begin
      if FTryWindowed then
      begin
        { LFB -> Windowed fallback (useful for buggy DPMI hosts like NTVDM) }
	LOG('Setting LFB mode failed; will try without LFB...');
        DisableLFB;
	Open(ATitle, AMode, APages);
	exit;
      end
      else
        raise TPTCError.Create('error setting VBE mode');
    end;
  end
  else
  begin
    if not SetVESAMode(FModesN[ModeFound].Index, false) then
      raise TPTCError.Create('error setting VBE mode');
  end;
  
  FVESACurrentMode := FModesN[ModeFound].Index;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  with FModes[FCurrentMode].Format do
=======
  with FModes[FCurrentMode].FFormat do
>>>>>>> graemeg/cpstrnew
=======
  with FModes[FCurrentMode].FFormat do
>>>>>>> graemeg/cpstrnew
=======
  with FModes[FCurrentMode].FFormat do
>>>>>>> graemeg/cpstrnew
=======
  with FModes[FCurrentMode].FFormat do
>>>>>>> origin/cpstrnew
    if (Bits = 8) and Direct and (R = $E0) and (G = $1C) and (B = $03) then
      SetRGB332Palette;

  with VBEModes[FVESACurrentMode] do
  begin
    FWidth := XResolution;
    FHeight := YResolution;
    if LFBUsed then
    begin
      FPitch := LFBBytesPerScanLine;
      FVideoPagesCount := LFBNumberOfImagePages + 1;
    end
    else
    begin
      FPitch := WindowedBytesPerScanline;
      FVideoPagesCount := WindowedNumberOfImagePages + 1;
    end;
    if FVideoPagesCount < 1 then
      FVideoPagesCount := 1;
    if (APages > 0) and (FVideoPagesCount > APages) then
      FVideoPagesCount := APages;
    FVideoPageSize := YResolution * FPitch;
    FVideoPageHeight := YResolution;
    FCurrentVideoPage := 0;
    if FVideoPagesCount > 1 then
      FNextVideoPage := 1
    else
      FNextVideoPage := 0;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FArea := TPTCArea.Create(0, 0, FWidth, FHeight);
  FClip := FArea;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  tmpa := TPTCArea.Create(0, 0, FWidth, FHeight);
  try
    FArea.Assign(tmpa);
    FClip.Assign(tmpa);
  finally
    tmpa.Free;
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

  FLFBNearPtrAccessAvailable := LFBNearPtrAccessAvailable;
  if not FLFBNearPtrAccessAvailable then
  begin
    FPrimary := GetMem(FHeight * FPitch);
    FillChar(FPrimary^, FHeight * FPitch, 0);
  end;

  if FLFBNearPtrAccessAvailable and (FVideoPagesCount > 1) then
  begin
    { clear video memory pages numbers 1..FVideoPagesCount-1 }
    FillChar((PByte(LFBNearPtrAccessPtr) + FVideoPageSize)^, FVideoPageSize * (FVideoPagesCount - 1), 0);
  end;

  FreeAndNil(FKeyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
  FKeyboard := TDosKeyboard.Create;
  FMouse := TDosMouse.Create(FWidth, FHeight);
  FEventQueue := TEventQueue.Create;

  { temporary platform dependent information fudge }
  FInformation := 'dos version x.xx.x'+#13+#10+'vesa version x.xx'+#13+#10+'vesa driver name xxxxx'+#13+#10+'display driver vendor xxxxx'+#13+#10+'certified driver? x'+#13+#10;

  { set open flag }
  FOpen := True;
end;

procedure TVESAConsole.Close;
begin
  if FOpen then
  begin
    if FLocked then
      raise TPTCError.Create('console is still locked');
    {flush all key presses}
    while KeyPressed do ReadKey;
    internal_close;
    FOpen := False;
  end;
end;

procedure TVESAConsole.Flush;
begin
  check_open;
  check_unlocked;
end;

procedure TVESAConsole.Finish;
begin
  check_open;
  check_unlocked;
end;

procedure TVESAConsole.Update;
var
  framebuffer: PInteger;
begin
  check_open;
  check_unlocked;
  if FVideoPagesCount = 1 then
  begin
    WaitRetraceSinglePage;
    if not FLFBNearPtrAccessAvailable then
    begin
      WriteToVideoMemory(FPrimary, 0, FPitch * FHeight);
    end;
  end
  else
  begin
    if not FLFBNearPtrAccessAvailable then
    begin
      WriteToVideoMemory(FPrimary, FNextVideoPage * FVideoPageSize, FPitch * FHeight);
    end;
    SetDisplayStart(0, FNextVideoPage * FVideoPageHeight, true);
    FCurrentVideoPage := FNextVideoPage;
    FNextVideoPage := FCurrentVideoPage + 1;
    if FNextVideoPage >= FVideoPagesCount then
      FNextVideoPage := 0;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVESAConsole.Update(AArea: IPTCArea);
=======
procedure TVESAConsole.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Update(const AArea: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  Update;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVESAConsole.Copy(ASurface: IPTCSurface);
=======
procedure TVESAConsole.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Copy(ASurface: TPTCBaseSurface);
>>>>>>> origin/cpstrnew
var
  Pixels: Pointer;
begin
  check_open;
  check_unlocked;
  Pixels := Lock;
  try
    ASurface.Load(Pixels, Width, Height, Pitch, Format, Palette);
    Unlock;
  except
    on error: TPTCError do
    begin
      Unlock;
      raise TPTCError.Create('failed to copy console to surface', error);
    end;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVESAConsole.Copy(ASurface: IPTCSurface;
                            ASource, ADestination: IPTCArea);
=======
procedure TVESAConsole.Copy(ASurface: TPTCBaseSurface;
                            const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Copy(ASurface: TPTCBaseSurface;
                            const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Copy(ASurface: TPTCBaseSurface;
                            const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Copy(ASurface: TPTCBaseSurface;
                            const ASource, ADestination: TPTCArea);
>>>>>>> origin/cpstrnew
var
  pixels: Pointer;
begin
  check_open;
  check_unlocked;
  Pixels := Lock;
  try
    ASurface.Load(Pixels, Width, Height, Pitch, Format, Palette, ASource, ADestination);
    Unlock;
  except
    on error: TPTCError do
    begin
      Unlock;
      raise TPTCError.Create('failed to copy console to surface', error);
    end;
  end;
end;

function TVESAConsole.Lock: Pointer;
var
  pixels: Pointer;
begin
  check_open;
  if FLocked then
    raise TPTCError.Create('console is already locked');

  FLocked := True;
  if FLFBNearPtrAccessAvailable then
    Result := PByte(LFBNearPtrAccessPtr) + (FNextVideoPage * FVideoPageSize)
  else
    Result := FPrimary;
end;

procedure TVESAConsole.Unlock;
begin
  check_open;
  if not FLocked then
    raise TPTCError.Create('console is not locked');

  FLocked := False;
end;

procedure TVESAConsole.Load(const APixels: Pointer;
                            AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            AFormat: IPTCFormat;
                            APalette: IPTCPalette);
var
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            const AFormat: TPTCFormat;
                            const APalette: TPTCPalette);
var
  Area_: TPTCArea;
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
  ConsolePixels: Pointer;
begin
  check_open;
  check_unlocked;
  if Clip.Equals(Area) then
  begin
    ConsolePixels := Lock;
    try
      try
        FCopy.Request(AFormat, Format);
        FCopy.Palette(APalette, Palette);
        FCopy.Copy(APixels, 0, 0, AWidth, AHeight, APitch, ConsolePixels, 0, 0,
                    Width, Height, Pitch);
      except
        on error: TPTCError do
        begin
          raise TPTCError.Create('failed to load pixels to console', error);
        end;
      end;
    finally
      Unlock;
    end;
  end
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, TPTCArea.Create(0, 0, Width, Height), Area);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  begin
    Area_ := TPTCArea.Create(0, 0, Width, Height);
    try
      Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area_, Area);
    finally
      Area_.Free;
    end;
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
end;

procedure TVESAConsole.Load(const APixels: Pointer;
                            AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            AFormat: IPTCFormat;
                            APalette: IPTCPalette;
                            ASource, ADestination: IPTCArea);
var
  ConsolePixels: Pointer;
  ClippedSource, ClippedDestination: IPTCArea;
begin
  check_open;
  check_unlocked;
  ConsolePixels := Lock;
  try
    try
      TPTCClipper.Clip(ASource, TPTCArea.Create(0, 0, AWidth, AHeight), ClippedSource, ADestination, Clip, ClippedDestination);
      FCopy.Request(AFormat, Format);
      FCopy.Palette(APalette, Palette);
      FCopy.Copy(APixels, ClippedSource.Left, ClippedSource.Top, ClippedSource.Width, ClippedSource.Height, APitch,
                 ConsolePixels, ClippedDestination.Left, ClippedDestination.Top, ClippedDestination.Width, ClippedDestination.Height, Pitch);
    except
      on error:TPTCError do
      begin
        raise TPTCError.Create('failed to load pixels to console area', error);
      end;
    end;
  finally
    Unlock;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            const AFormat: TPTCFormat;
                            const APalette: TPTCPalette;
                            const ASource, ADestination: TPTCArea);
var
  ConsolePixels: Pointer;
  ClippedSource: TPTCArea = nil;
  ClippedDestination: TPTCArea = nil;
  tmp: TPTCArea;
begin
  check_open;
  check_unlocked;
  try
    ClippedSource := TPTCArea.Create;
    ClippedDestination := TPTCArea.Create;
    ConsolePixels := Lock;
    try
      try
        tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
        try
          TPTCClipper.Clip(ASource, tmp, ClippedSource, ADestination, Clip, ClippedDestination);
        finally
          tmp.Free;
        end;
        FCopy.Request(AFormat, Format);
        FCopy.Palette(APalette, Palette);
        FCopy.Copy(APixels, ClippedSource.Left, ClippedSource.Top, ClippedSource.Width, ClippedSource.Height, APitch,
                    ConsolePixels, ClippedDestination.Left, ClippedDestination.Top, ClippedDestination.Width, ClippedDestination.Height, Pitch);
      except
        on error:TPTCError do
        begin
          raise TPTCError.Create('failed to load pixels to console area', error);
        end;
      end;
    finally
      Unlock;
    end;
  finally
    ClippedSource.Free;
    ClippedDestination.Free;
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
  end;
end;

procedure TVESAConsole.Save(APixels: Pointer;
                            AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            AFormat: IPTCFormat;
                            APalette: IPTCPalette);
var
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            const AFormat: TPTCFormat;
                            const APalette: TPTCPalette);
var
  Area_: TPTCArea;
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
  ConsolePixels: Pointer;
begin
  check_open;
  check_unlocked;
  if Clip.Equals(Area) then
  begin
    ConsolePixels := Lock;
    try
      try
        FCopy.Request(Format, AFormat);
        FCopy.Palette(Palette, APalette);
        FCopy.Copy(ConsolePixels, 0, 0, Width, Height, Pitch, APixels, 0, 0,
                    AWidth, AHeight, APitch);
      except
        on error: TPTCError do
        begin
          raise TPTCError.Create('failed to save console pixels', error);
        end;
      end;
    finally
      Unlock;
    end;
  end
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Save(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area, TPTCArea.Create(0, 0, Width, Height));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  begin
    Area_ := TPTCArea.Create(0, 0, Width, Height);
    try
      Save(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area, Area_);
    finally
      Area_.Free;
    end;
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
end;

procedure TVESAConsole.Save(APixels: Pointer;
                            AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            AFormat: IPTCFormat;
                            APalette: IPTCPalette;
                            ASource, ADestination: IPTCArea);
var
  ConsolePixels: Pointer;
  ClippedSource, ClippedDestination: IPTCArea;
begin
  check_open;
  check_unlocked;
  ConsolePixels := Lock;
  try
    try
      TPTCClipper.Clip(ASource, Clip, ClippedSource, ADestination, TPTCArea.Create(0, 0, AWidth, AHeight), ClippedDestination);
      FCopy.Request(Format, AFormat);
      FCopy.Palette(Palette, APalette);
      FCopy.Copy(ConsolePixels, ClippedSource.Left, ClippedSource.Top, ClippedSource.Width, ClippedSource.Height, Pitch,
                 APixels, ClippedDestination.Left, ClippedDestination.Top, ClippedDestination.Width, ClippedDestination.Height, APitch);
    except
      on error:TPTCError do
      begin
        raise TPTCError.Create('failed to save console area pixels', error);
      end;
    end;
  finally
    Unlock;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            const AFormat: TPTCFormat;
                            const APalette: TPTCPalette;
                            const ASource, ADestination: TPTCArea);
var
  ConsolePixels: Pointer;
  ClippedSource: TPTCArea = nil;
  ClippedDestination: TPTCArea = nil;
  tmp: TPTCArea;
begin
  check_open;
  check_unlocked;
  try
    ClippedSource := TPTCArea.Create;
    ClippedDestination := TPTCArea.Create;
    ConsolePixels := Lock;
    try
      try
        tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
        try
          TPTCClipper.Clip(ASource, Clip, ClippedSource, ADestination, tmp, ClippedDestination);
        finally
          tmp.Free;
        end;
        FCopy.Request(Format, AFormat);
        FCopy.Palette(Palette, APalette);
        FCopy.Copy(ConsolePixels, ClippedSource.Left, ClippedSource.Top, ClippedSource.Width, ClippedSource.Height, Pitch,
                    APixels, ClippedDestination.Left, ClippedDestination.Top, ClippedDestination.Width, ClippedDestination.Height, APitch);
      except
        on error:TPTCError do
        begin
          raise TPTCError.Create('failed to save console area pixels', error);
        end;
      end;
    finally
      Unlock;
    end;
  finally
    ClippedSource.Free;
    ClippedDestination.Free;
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
  end;
end;

procedure TVESAConsole.Clear;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Color: IPTCColor;
=======
  tmp: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  tmp: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  tmp: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  tmp: TPTCColor;
>>>>>>> origin/cpstrnew
begin
  check_open;
  check_unlocked;
  if Format.Direct then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Color := TPTCColor.Create(0, 0, 0, 0)
  else
    Color := TPTCColor.Create(0);
  Clear(Color);
end;

procedure TVESAConsole.Clear(AColor: IPTCColor);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    tmp := TPTCColor.Create(0, 0, 0, 0)
  else
    tmp := TPTCColor.Create(0);
  try
    Clear(tmp);
  finally
    tmp.Free;
  end;
end;

procedure TVESAConsole.Clear(const AColor: TPTCColor);
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
var
  tmp: TPTCArea;
begin
  check_open;
  check_unlocked;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Clear(AColor, TPTCArea.Create);
end;

procedure TVESAConsole.Clear(AColor: IPTCColor;
                             AArea: IPTCArea);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  tmp := TPTCArea.Create;
  try
    Clear(AColor, tmp);
  finally
    tmp.Free;
  end;
end;

procedure TVESAConsole.Clear(const AColor: TPTCColor;
                             const AArea: TPTCArea);
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
begin
  check_open;
  check_unlocked;
  {...}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVESAConsole.Palette(APalette: IPTCPalette);
=======
procedure TVESAConsole.Palette(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Palette(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Palette(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Palette(const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
  check_open;

  if Format.Indexed then
  begin
    FPalette.Load(APalette.Data);
    SetPalette(APalette.Data, 0, 256);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVESAConsole.Palette: IPTCPalette;
=======
function TVESAConsole.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.Palette: TPTCPalette;
>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := FPalette;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVESAConsole.Clip(AArea: IPTCArea);
=======
procedure TVESAConsole.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TVESAConsole.Clip(const AArea: TPTCArea);
>>>>>>> origin/cpstrnew
var
  tmp: TPTCArea;
begin
  check_open;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FClip := TPTCClipper.Clip(AArea, FArea);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  tmp := TPTCClipper.Clip(AArea, FArea);
  try
    FClip.Assign(tmp);
  finally
    tmp.Free;
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
end;

function TVESAConsole.GetWidth: Integer;
begin
  check_open;
  Result := FWidth;
end;

function TVESAConsole.GetHeight: Integer;
begin
  check_open;
  Result := FHeight;
end;

function TVESAConsole.GetPitch: Integer;
begin
  check_open;
  Result := FPitch;
end;

function TVESAConsole.GetPages: Integer;
begin
  check_open;
  Result := 2;{FPrimary.pages;}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVESAConsole.GetArea: IPTCArea;
=======
function TVESAConsole.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.GetArea: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := FArea;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVESAConsole.Clip: IPTCArea;
=======
function TVESAConsole.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.Clip: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := FClip;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVESAConsole.GetFormat: IPTCFormat;
=======
function TVESAConsole.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.GetFormat: TPTCFormat;
>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := FModes[FCurrentMode].Format;
end;

function TVESAConsole.GetName: string;
begin
  Result := 'VESA';
end;

function TVESAConsole.GetTitle: string;
begin
  Result := FTitle;
end;

function TVESAConsole.GetInformation: string;
begin
  Result := FInformation;
end;

procedure TVESAConsole.internal_close;
begin
  FreeMemAndNil(FPrimary);
  FreeAndNil(FKeyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
  RestoreTextMode;
end;

procedure TVESAConsole.HandleEvents;
begin
  FKeyboard.GetPendingEvents(FEventQueue);
  FMouse.GetPendingEvents(FEventQueue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVESAConsole.NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
begin
  check_open;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TVESAConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
begin
  check_open;

  FreeAndNil(AEvent);
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
  repeat
    { get events }
    HandleEvents;

    { try to find an event that matches the EventMask }
    AEvent := FEventQueue.NextEvent(AEventMask);
  until (not AWait) or (AEvent <> nil);
  Result := AEvent <> nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVESAConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent;
=======
function TVESAConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TVESAConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> origin/cpstrnew
begin
  check_open;

  repeat
    { get events }
    HandleEvents;

    { try to find an event that matches the EventMask }
    Result := FEventQueue.PeekEvent(AEventMask);
  until (not AWait) or (Result <> nil);
end;

procedure TVESAConsole.check_open;
begin
  if not FOpen then
    raise TPTCError.Create('console is not open');
end;

procedure TVESAConsole.check_unlocked;
begin
  if FLocked then
    raise TPTCError.Create('console is not unlocked');
end;
