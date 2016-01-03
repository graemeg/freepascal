{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

{$INLINE ON}

Class Function TPTCClipper.Clip(Const AArea, AClip : TPTCArea) : TPTCArea;

Var
  left, top, right, bottom : Integer;
  clip_left, clip_top, clip_right, clip_bottom : Integer;

Begin
  { get in coordinates }
  left   := AArea.Left;
  top    := AArea.Top;
  right  := AArea.Right;
  bottom := AArea.Bottom;

  { get clip coordinates }
  clip_left   := AClip.Left;
  clip_top    := AClip.Top;
  clip_right  := AClip.Right;
  clip_bottom := AClip.Bottom;

  { clip left }
  If left < clip_left Then
    left := clip_left;
  If left > clip_right Then
    left := clip_right;

  { clip top }
  If top < clip_top Then
    top := clip_top;
  If top > clip_bottom Then
    top := clip_bottom;

  { clip right }
  If right < clip_left Then
    right := clip_left;
  If right > clip_right Then
    right := clip_right;

  { clip bottom }
  If bottom < clip_top Then
    bottom := clip_top;
  If bottom > clip_bottom Then
    bottom := clip_bottom;

  Result := TPTCArea.Create(Left, Top, Right, Bottom);
End;

{ clip floating point area against a floating point clip area }
Procedure TPTCClipper_clip(Var left, top, right, bottom : Real;
                           clip_left, clip_top, clip_right, clip_bottom : Real); Inline;

Begin
  { clip left }
  If left < clip_left Then
    left := clip_left;
  If left > clip_right Then
    left := clip_right;
  { clip top }
  If top < clip_top Then
    top := clip_top;
  If top > clip_bottom Then
    top := clip_bottom;
  { clip right }
  If right < clip_left Then
    right := clip_left;
  If right > clip_right Then
    right := clip_right;
  { clip bottom }
  If bottom < clip_top Then
    bottom := clip_top;
  If bottom > clip_bottom Then
    bottom := clip_bottom;
End;

{ clip floating point area against clip area }
Procedure TPTCClipper_clip(Var left, top, right, bottom : Real; Const _clip : TPTCArea); Inline;

Var
  clip_left, clip_top, clip_right, clip_bottom : Real;

Begin
  { get floating point clip area }
  clip_left := _clip.left;
  clip_top := _clip.top;
  clip_right := _clip.right;
  clip_bottom := _clip.bottom;
  { clip floating point area against floating point clip area }
  TPTCClipper_clip(left, top, right, bottom, clip_left, clip_top, clip_right, clip_bottom);
End;

{ snap a floating point area to integer coordinates }
Procedure TPTCClipper_round(Var left, top, right, bottom : Real); Inline;

Begin
  left := Round(left);
  top := Round(top);
  right := Round(right);
  bottom := Round(bottom);
End;

Class Procedure TPTCClipper.Clip(Const ASource, AClipSource, AClippedSource,
                                 ADestination, AClipDestination,
                                 AClippedDestination : TPTCArea);

Var
  tmp1, tmp2 : TPTCArea;

  source_left, source_top, source_right, source_bottom : Real;
  clipped_source_left, clipped_source_top, clipped_source_right,
  clipped_source_bottom : Real;
  source_delta_left, source_delta_top, source_delta_right,
  source_delta_bottom : Real;
  source_to_destination_x, source_to_destination_y : Real;
  destination_left, destination_top, destination_right,
  destination_bottom : Real;
  adjusted_destination_left, adjusted_destination_top,
  adjusted_destination_right, adjusted_destination_bottom : Real;
  clipped_destination_left, clipped_destination_top,
  clipped_destination_right, clipped_destination_bottom : Real;
  destination_delta_left, destination_delta_top, destination_delta_right,
  destination_delta_bottom : Real;
  destination_to_source_x, destination_to_source_y : Real;
  adjusted_source_left, adjusted_source_top, adjusted_source_right,
  adjusted_source_bottom : Real;

Begin
  tmp1 := Nil;
  tmp2 := Nil;
  Try
    { expand source area to floating point }
    source_left   := ASource.Left;
    source_top    := ASource.Top;
    source_right  := ASource.Right;
    source_bottom := ASource.Bottom;

    { setup clipped source area }
    clipped_source_left := source_left;
    clipped_source_top := source_top;
    clipped_source_right := source_right;
    clipped_source_bottom := source_bottom;

    { perform clipping on floating point source area }
    TPTCClipper_clip(clipped_source_left, clipped_source_top, clipped_source_right,
                     clipped_source_bottom, AClipSource);

    { check for early source area clipping exit }
    If (clipped_source_left = clipped_source_right) Or
       (clipped_source_top = clipped_source_bottom) Then
    Begin
      { clipped area is zero }
      tmp1 := TPTCArea.Create(0, 0, 0, 0);
      AClippedSource.Assign(tmp1);
      AClippedDestination.Assign(tmp1);
      Exit;
    End;

    { calculate deltas in source clip }
    source_delta_left := clipped_source_left - source_left;
    source_delta_top := clipped_source_top - source_top;
    source_delta_right := clipped_source_right - source_right;
    source_delta_bottom := clipped_source_bottom - source_bottom;

    { calculate ratio of source area to destination area }
    source_to_destination_x := ADestination.Width / ASource.Width;
    source_to_destination_y := ADestination.Height / ASource.Height;

    { expand destination area to floating point }
    destination_left   := ADestination.Left;
    destination_top    := ADestination.Top;
    destination_right  := ADestination.Right;
    destination_bottom := ADestination.Bottom;

    { calculate adjusted destination area }
    adjusted_destination_left := destination_left + source_delta_left * source_to_destination_x;
    adjusted_destination_top := destination_top + source_delta_top * source_to_destination_y;
    adjusted_destination_right := destination_right + source_delta_right * source_to_destination_x;
    adjusted_destination_bottom := destination_bottom + source_delta_bottom * source_to_destination_y;

    { setup clipped destination area }
    clipped_destination_left := adjusted_destination_left;
    clipped_destination_top := adjusted_destination_top;
    clipped_destination_right := adjusted_destination_right;
    clipped_destination_bottom := adjusted_destination_bottom;

    { perform clipping on floating point destination area }
    TPTCClipper_clip(clipped_destination_left, clipped_destination_top,
                     clipped_destination_right, clipped_destination_bottom, AClipDestination);

    { check for early destination area clipping exit }
    If (clipped_destination_left = clipped_destination_right) Or
       (clipped_destination_top = clipped_destination_bottom) Then
    Begin
      { clipped area is zero }
      tmp1 := TPTCArea.Create(0, 0, 0, 0);
      AClippedSource.Assign(tmp1);
      AClippedDestination.Assign(tmp1);
      Exit;
    End;

    { calculate deltas in destination clip }
    destination_delta_left := clipped_destination_left - adjusted_destination_left;
    destination_delta_top := clipped_destination_top - adjusted_destination_top;
    destination_delta_right := clipped_destination_right - adjusted_destination_right;
    destination_delta_bottom := clipped_destination_bottom - adjusted_destination_bottom;

    { calculate ratio of destination area to source area }
    destination_to_source_x := 1 / source_to_destination_x;
    destination_to_source_y := 1 / source_to_destination_y;

    { calculate adjusted source area }
    adjusted_source_left := clipped_source_left + destination_delta_left * destination_to_source_x;
    adjusted_source_top := clipped_source_top + destination_delta_top * destination_to_source_y;
    adjusted_source_right := clipped_source_right + destination_delta_right * destination_to_source_x;
    adjusted_source_bottom := clipped_source_bottom + destination_delta_bottom * destination_to_source_y;

    { assign adjusted source to clipped source }
    clipped_source_left := adjusted_source_left;
    clipped_source_top := adjusted_source_top;
    clipped_source_right := adjusted_source_right;
    clipped_source_bottom := adjusted_source_bottom;

    { round clipped areas to integer coordinates }
    TPTCClipper_round(clipped_source_left, clipped_source_top,
                      clipped_source_right, clipped_source_bottom);
    TPTCClipper_round(clipped_destination_left, clipped_destination_top,
                      clipped_destination_right, clipped_destination_bottom);

    { construct clipped area rectangles from rounded floating point areas }
    tmp1 := TPTCArea.Create(Trunc(clipped_source_left),
                            Trunc(clipped_source_top),
                            Trunc(clipped_source_right),
                            Trunc(clipped_source_bottom));
    tmp2 := TPTCArea.Create(Trunc(clipped_destination_left),
                            Trunc(clipped_destination_top),
                            Trunc(clipped_destination_right),
                            Trunc(clipped_destination_bottom));
    AClippedSource.Assign(tmp1);
    AClippedDestination.Assign(tmp2);
  Finally
    tmp1.Free;
    tmp2.Free;
  End;
End;
