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

Procedure DirectXCheck(result : HRESULT);

Begin
  If result = DD_OK Then
    Exit;
{ $IFDEF __PTC_DIRECTX_ERROR_STRINGS__}
  Case result Of
    DDERR_ALREADYINITIALIZED	       : Raise TPTCError.Create('DDERR_ALREADYINITIALIZED');
    DDERR_CANNOTATTACHSURFACE	       : Raise TPTCError.Create('DDERR_CANNOTATTACHSURFACE');
    DDERR_CANNOTDETACHSURFACE	       : Raise TPTCError.Create('DDERR_CANNOTDETACHSURFACE');
    DDERR_CURRENTLYNOTAVAIL	       : Raise TPTCError.Create('DDERR_CURRENTLYNOTAVAIL');
    DDERR_EXCEPTION		       : Raise TPTCError.Create('DDERR_EXCEPTION');
    DDERR_GENERIC		       : Raise TPTCError.Create('DDERR_GENERIC');
    DDERR_HEIGHTALIGN		       : Raise TPTCError.Create('DDERR_HEIGHTALIGN');
    DDERR_INCOMPATIBLEPRIMARY	       : Raise TPTCError.Create('DDERR_INCOMPATIBLEPRIMARY');
    DDERR_INVALIDCAPS		       : Raise TPTCError.Create('DDERR_INVALIDCAPS');
    DDERR_INVALIDCLIPLIST	       : Raise TPTCError.Create('DDERR_INVALIDCLIPLIST');
    DDERR_INVALIDMODE		       : Raise TPTCError.Create('DDERR_INVALIDMODE');
    DDERR_INVALIDOBJECT 	       : Raise TPTCError.Create('DDERR_INVALIDOBJECT');
    DDERR_INVALIDPARAMS 	       : Raise TPTCError.Create('DDERR_INVALIDPARAMS');
    DDERR_INVALIDPIXELFORMAT	       : Raise TPTCError.Create('DDERR_INVALIDPIXELFORMAT');
    DDERR_INVALIDRECT		       : Raise TPTCError.Create('DDERR_INVALIDRECT');
    DDERR_LOCKEDSURFACES	       : Raise TPTCError.Create('DDERR_LOCKEDSURFACES');
    DDERR_NO3D			       : Raise TPTCError.Create('DDERR_NO3D');
    DDERR_NOALPHAHW		       : Raise TPTCError.Create('DDERR_NOALPHAHW');
    DDERR_NOCLIPLIST		       : Raise TPTCError.Create('DDERR_CLIPLIST');
    DDERR_NOCOLORCONVHW 	       : Raise TPTCError.Create('DDERR_NOCOLORCONVHW');
    DDERR_NOCOOPERATIVELEVELSET        : Raise TPTCError.Create('DDERR_NOCOOPERATIVELEVELSET');
    DDERR_NOCOLORKEY		       : Raise TPTCError.Create('DDERR_NOCOLORKEY');
    DDERR_NOCOLORKEYHW		       : Raise TPTCError.Create('DDERR_NOCOLORKEYHW');
    DDERR_NODIRECTDRAWSUPPORT	       : Raise TPTCError.Create('DDERR_NODIRECTDRAWSUPPORT');
    DDERR_NOEXCLUSIVEMODE	       : Raise TPTCError.Create('DDERR_NOEXCLUSIVEMODE');
    DDERR_NOFLIPHW		       : Raise TPTCError.Create('DDERR_NOFLIPHW');
    DDERR_NOGDI 		       : Raise TPTCError.Create('DDERR_NOGDI');
    DDERR_NOMIRRORHW		       : Raise TPTCError.Create('DDERR_NOMIRRORHW');
    DDERR_NOTFOUND		       : Raise TPTCError.Create('DDERR_NOTFOUND');
    DDERR_NOOVERLAYHW		       : Raise TPTCError.Create('DDERR_NOOVERLAYHW');
    DDERR_NORASTEROPHW		       : Raise TPTCError.Create('DDERR_NORASTEROPHW');
    DDERR_NOROTATIONHW		       : Raise TPTCError.Create('DDERR_NOROTATIONHW');
    DDERR_NOSTRETCHHW		       : Raise TPTCError.Create('DDERR_NOSTRETCHHW');
    DDERR_NOT4BITCOLOR		       : Raise TPTCError.Create('DDERR_NOT4BITCOLOR');
    DDERR_NOT4BITCOLORINDEX	       : Raise TPTCError.Create('DDERR_NOT4BITCOLORINDEX');
    DDERR_NOT8BITCOLOR		       : Raise TPTCError.Create('DDERR_NOT8BITCOLOR');
    DDERR_NOTEXTUREHW		       : Raise TPTCError.Create('DDERR_NOTEXTUREHW');
    DDERR_NOVSYNCHW		       : Raise TPTCError.Create('DDERR_NOVSYNCHW');
    DDERR_NOZBUFFERHW		       : Raise TPTCError.Create('DDERR_NOZBUFFERHW');
    DDERR_NOZOVERLAYHW		       : Raise TPTCError.Create('DDERR_NOZOVERLAYHW');
    DDERR_OUTOFCAPS		       : Raise TPTCError.Create('DDERR_OUTOFCAPS');
    DDERR_OUTOFMEMORY		       : Raise TPTCError.Create('DDERR_OUTOFMEMORY');
    DDERR_OUTOFVIDEOMEMORY	       : Raise TPTCError.Create('DDERR_OUTOFVIDEOMEMORY');
    DDERR_OVERLAYCANTCLIP	       : Raise TPTCError.Create('DDERR_OVERLAYCANTCLIP');
    DDERR_OVERLAYCOLORKEYONLYONEACTIVE : Raise TPTCError.Create('DDERR_OVERLAYCOLORKEYONLYONEACTIVE');
    DDERR_PALETTEBUSY		       : Raise TPTCError.Create('DDERR_PALETTEBUSY');
    DDERR_COLORKEYNOTSET	       : Raise TPTCError.Create('DDERR_COLORKEYNOTSET');
    DDERR_SURFACEALREADYATTACHED       : Raise TPTCError.Create('DDERR_SURFACEALREADYATTACHED');
    DDERR_SURFACEALREADYDEPENDENT      : Raise TPTCError.Create('DDERR_SURFACEALREADYDEPENDENT');
    DDERR_SURFACEBUSY		       : Raise TPTCError.Create('DDERR_SURFACEBUSY');
    DDERR_CANTLOCKSURFACE	       : Raise TPTCError.Create('DDERR_CANTLOCKSURFACE');
    DDERR_SURFACEISOBSCURED	       : Raise TPTCError.Create('DDERR_SURFACEISOBSCURED');
    DDERR_SURFACELOST		       : Raise TPTCError.Create('DDERR_SURFACELOST');
    DDERR_SURFACENOTATTACHED	       : Raise TPTCError.Create('DDERR_SURFACENOTATTACHED');
    DDERR_TOOBIGHEIGHT		       : Raise TPTCError.Create('DDERR_TOOBIGHEIGHT');
    DDERR_TOOBIGSIZE		       : Raise TPTCError.Create('DDERR_TOOBIGSIZE');
    DDERR_TOOBIGWIDTH		       : Raise TPTCError.Create('DDERR_TOOBIGWIDTH');
    DDERR_UNSUPPORTED		       : Raise TPTCError.Create('DDERR_UNSUPPORTED');
    DDERR_UNSUPPORTEDFORMAT	       : Raise TPTCError.Create('DDERR_UNSUPPORTEDFORMAT');
    DDERR_UNSUPPORTEDMASK	       : Raise TPTCError.Create('DDERR_UNSUPPORTEDMASK');
    DDERR_VERTICALBLANKINPROGRESS      : Raise TPTCError.Create('DDERR_VERTICALBLANKINPROGRESS');
    DDERR_WASSTILLDRAWING	       : Raise TPTCError.Create('DDERR_WASSTILLDRAWING');
    DDERR_XALIGN		       : Raise TPTCError.Create('DDERR_XALIGN');
    DDERR_INVALIDDIRECTDRAWGUID        : Raise TPTCError.Create('DDERR_INVALIDDIRECTDRAWGUID');
    DDERR_DIRECTDRAWALREADYCREATED     : Raise TPTCError.Create('DDERR_DIRECTDRAWALREADYCREATED');
    DDERR_NODIRECTDRAWHW	       : Raise TPTCError.Create('DDERR_NODIRECTDRAWHW');
    DDERR_PRIMARYSURFACEALREADYEXISTS  : Raise TPTCError.Create('DDERR_PRIMARYSURFACEALREADYEXISTS');
    DDERR_NOEMULATION		       : Raise TPTCError.Create('DDERR_NOEMULATION');
    DDERR_REGIONTOOSMALL	       : Raise TPTCError.Create('DDERR_REGIONTOOSMALL');
    DDERR_CLIPPERISUSINGHWND	       : Raise TPTCError.Create('DDERR_CLIPPERISUSINGHWND');
    DDERR_NOCLIPPERATTACHED	       : Raise TPTCError.Create('DDERR_NOCLIPPERATTACHED');
    DDERR_NOHWND		       : Raise TPTCError.Create('DDERR_NOHWND');
    DDERR_HWNDSUBCLASSED	       : Raise TPTCError.Create('DDERR_HWNDSUBCLASSED');
    DDERR_HWNDALREADYSET	       : Raise TPTCError.Create('DDERR_HWNDALREADYSET');
    DDERR_NOPALETTEATTACHED	       : Raise TPTCError.Create('DDERR_NOPALETTEATTACHED');
    DDERR_NOPALETTEHW		       : Raise TPTCError.Create('DDERR_NOPALETTEHW');
    DDERR_BLTFASTCANTCLIP	       : Raise TPTCError.Create('DDERR_BLTFASTCANTCLIP');
    DDERR_NOBLTHW		       : Raise TPTCError.Create('DDERR_NOBLTHW');
    DDERR_NODDROPSHW		       : Raise TPTCError.Create('DDERR_NODDROPSHW');
    DDERR_OVERLAYNOTVISIBLE	       : Raise TPTCError.Create('DDERR_OVERLAYNOTVISIBLE');
    DDERR_NOOVERLAYDEST 	       : Raise TPTCError.Create('DDERR_NOOVERLAYDEST');
    DDERR_INVALIDPOSITION	       : Raise TPTCError.Create('DDERR_INVALIDPOSITION');
    DDERR_NOTAOVERLAYSURFACE	       : Raise TPTCError.Create('DDERR_NOTAOVERLAYSURFACE');
    DDERR_EXCLUSIVEMODEALREADYSET      : Raise TPTCError.Create('DDERR_EXCLUSIVEMODEALREADYSET');
    DDERR_NOTFLIPPABLE		       : Raise TPTCError.Create('DDERR_NOTFLIPPABLE');
    DDERR_CANTDUPLICATE 	       : Raise TPTCError.Create('DDERR_CANTDUPLICATE');
    DDERR_NOTLOCKED		       : Raise TPTCError.Create('DDERR_NOTLOCKED');
    DDERR_CANTCREATEDC		       : Raise TPTCError.Create('DDERR_CANTCREATEDC');
    DDERR_NODC			       : Raise TPTCError.Create('DDERR_NODC');
    DDERR_WRONGMODE		       : Raise TPTCError.Create('DDERR_WRONGMODE');
    DDERR_IMPLICITLYCREATED	       : Raise TPTCError.Create('DDERR_IMPLICITLYCREATED');
    DDERR_NOTPALETTIZED 	       : Raise TPTCError.Create('DDERR_NOPALETTIZED');
    DDERR_UNSUPPORTEDMODE	       : Raise TPTCError.Create('DDERR_UNSUPPORTEDMODE');
    DDERR_NOMIPMAPHW		       : Raise TPTCError.Create('DDERR_NOMIPMAPHW');
    DDERR_INVALIDSURFACETYPE	       : Raise TPTCError.Create('DDERR_INVALIDSURFACETYPE');
    DDERR_DCALREADYCREATED	       : Raise TPTCError.Create('DDERR_DCALREADYCREATED');
    DDERR_CANTPAGELOCK		       : Raise TPTCError.Create('DDERR_CANTPAGELOCK');
    DDERR_CANTPAGEUNLOCK	       : Raise TPTCError.Create('DDERR_CANTPAGEUNLOCK');
    DDERR_NOTPAGELOCKED 	       : Raise TPTCError.Create('DDERR_NOTPAGELOCKED');
    DDERR_NOTINITIALIZED	       : Raise TPTCError.Create('DDERR_NOTINITIALIZED');
  End;
{ $ENDIF}
  Raise TPTCError.Create('DDERR $' + HexStr(result, 8));
End;

Procedure DirectXCheck(result : HRESULT; Const message : String);

Begin
  Try
    DirectXCheck(result);
  Except
    On error : TPTCError Do
      Raise TPTCError.Create(message, error);
  End;
End;
