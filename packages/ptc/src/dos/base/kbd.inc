Constructor TDosKeyboard.Create;

Begin
  { defaults }
  m_key := False;
  m_head := 0;
  m_tail := 0;
End;

Destructor TDosKeyboard.Destroy;

Begin
  Inherited Destroy;
End;

Procedure TDosKeyboard.internal_ReadKey(k : TPTCKey);

Var
  read : TPTCKey;

Begin
  While Not ready Do;
  read := remove;
  Try
    k.ASSign(read);
  Finally
    read.Free;
  End;
End;

Function TDosKeyboard.internal_PeekKey(k : TPTCKey) : Boolean;

Begin
  Result := ready;
  If Result = True Then
    k.ASSign(m_buffer[m_tail]);
End;

Procedure TDosKeyboard.insert(_key : TPTCKey);

Begin
  { check for overflow }
  If (m_head <> (m_tail - 1)) And
    ((m_tail <> 0) Or (m_head <> High(m_buffer))) Then
  Begin
    { insert key at head }
    m_buffer[m_head] := _key;

    { increase head }
    Inc(m_head);

    { wrap head from end to start }
    If m_head > High(m_buffer) Then
      m_head := Low(m_buffer);
  End;
End;

Function TDosKeyboard.remove : TPTCKey;

Begin
  { return key data from tail }
  remove := m_buffer[m_tail];

  { increase tail }
  Inc(m_tail);

  { wrap tail from end to start }
  If m_tail > High(m_buffer) Then
    m_tail := Low(m_buffer);
End;

Function TDosKeyboard.ready : Boolean;

Var
  c : Integer;
  Ch, Ex : Char;

Begin
  If KeyPressed Then
  Begin
    Ch := ReadKey;
    If Ch = #0 Then
      Ex := ReadKey
    Else
      Ex := #0;
    If Ch <> #0 Then
    Begin
      Ch := UpCase(Ch);
      c := Ord(Ch);
    End
    Else
    Begin
      Case Ord(Ex) Of
        59 : c := PTCKEY_F1;
        60 : c := PTCKEY_F2;
        61 : c := PTCKEY_F3;
        62 : c := PTCKEY_F4;
        63 : c := PTCKEY_F5;
        64 : c := PTCKEY_F6;
        65 : c := PTCKEY_F7;
        66 : c := PTCKEY_F8;
        67 : c := PTCKEY_F9;
        68 : c := PTCKEY_F10;
        71 : c := PTCKEY_HOME;
        72 : c := PTCKEY_UP;
        73 : c := PTCKEY_PAGEUP;
        75 : c := PTCKEY_LEFT;
        76 : c := PTCKEY_NUMPAD5;
        77 : c := PTCKEY_RIGHT;
        79 : c := PTCKEY_END;
        80 : c := PTCKEY_DOWN;
        81 : c := PTCKEY_PAGEDOWN;
        82 : c := PTCKEY_INSERT;
        83 : c := PTCKEY_DELETE;
        133 : c := PTCKEY_F11;
        134 : c := PTCKEY_F12;
      End;
    End;
    insert(TPTCKey.Create(c, False, False, False, True));
    insert(TPTCKey.Create(c, False, False, False, False));
  End;
  ready := m_head <> m_tail;
End;
