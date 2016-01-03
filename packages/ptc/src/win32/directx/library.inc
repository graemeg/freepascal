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

Constructor TDirectXLibrary.Create;

Var
  DirectDrawCreate : TDirectDrawCreate;
  IID_IDirectDraw2 : GUID;

Begin
  { defaults }
  m_lpDD := Nil;
  m_lpDD2 := Nil;
  m_library := 0;

  Try
    LOG('loading ddraw.dll');
    m_library := LoadLibrary('ddraw.dll');
    If m_library = 0 Then
      Raise TPTCError.Create('could not load ddraw.dll');
    LOG('importing DirectDrawCreate');
    DirectDrawCreate := TDirectDrawCreate(GetProcAddress(m_library, 'DirectDrawCreate'));
    If DirectDrawCreate = Nil Then
      Raise TPTCError.Create('could not get address of DirectDrawCreate');
    LOG('creating lpDD');
    DirectXCheck(DirectDrawCreate(Nil, @m_lpDD, Nil));
    With IID_IDirectDraw2 Do
    Begin
      Data1 := $B3A6F3E0;
      Data2 := $2B43;
      Data3 := $11CF;
      Data4[0] := $A2;
      Data4[1] := $DE;
      Data4[2] := $00;
      Data4[3] := $AA;
      Data4[4] := $00;
      Data4[5] := $B9;
      Data4[6] := $33;
      Data4[7] := $56;
    End;
    LOG('querying lpDD2');
    DirectXCheck(m_lpDD^.lpVtbl^.QueryInterface(m_lpDD, @IID_IDirectDraw2, @m_lpDD2));
  Except
    On error : TPTCError Do
    Begin
      { close }
      close;

      { rethrow }
      Raise TPTCError.Create('could not initialize ddraw', error);
    End;
  End;
End;

Destructor TDirectXLibrary.Destroy;

Begin
  close;
  Inherited Destroy;
End;

Procedure TDirectXLibrary.close;

Begin
  If m_lpDD2 <> Nil Then
  Begin
    LOG('releasing lpDD2');
    m_lpDD2^.lpVtbl^.Release(m_lpDD2);
    m_lpDD2 := Nil;
  End;
  If m_lpDD <> Nil Then
  Begin
    LOG('releasing lpDD');
    m_lpDD^.lpVtbl^.Release(m_lpDD);
    m_lpDD := Nil;
  End;
  If m_library <> 0 Then
  Begin
    LOG('closing ddraw.dll');
    FreeLibrary(m_library);
    m_library := 0;
  End;
End;
