{
    This file is part of the Free Pascal run time library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 2010, 2011, 2013 by Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (c) 2010 by Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 2010 by Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew

    This file implements keyboard input support for ptcgraph

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit ptccrt;

{$MODE objfpc}
{$DEFINE HasCRT}

{$IFDEF WinCE}
  {$UNDEF HasCRT}
{$ENDIF WinCE}

interface

{$IFDEF HasCRT}
uses
  crt;
{$ENDIF HasCRT}

type
{$IFDEF HasCRT}
  tcrtcoord = crt.tcrtcoord;
{$ELSE HasCRT}
  tcrtcoord = 1..255;
{$ENDIF HasCRT}

var
  DirectVideo: Boolean {$IFDEF HasCRT}absolute crt.DirectVideo{$ENDIF HasCRT};
  TextAttr: Byte {$IFDEF HasCRT}absolute crt.TextAttr{$ENDIF HasCRT};

function KeyPressed: Boolean;
function ReadKey: Char;
procedure ClrScr;
procedure ClrEol;
procedure GotoXY(X, Y: tcrtcoord);
procedure TextColor(Color: Byte);
procedure TextBackground(Color: Byte);
procedure Delay(MS: Word);
procedure Sound(HZ: Word);
procedure NoSound;

implementation

uses
  ptcgraph, ptc, ptcwrapper
  {$IFDEF UNIX}
  , baseunix
  {$ENDIF UNIX}
  {$IF defined(Win32) or defined(Win64) or defined(WinCE)}
  , windows
  {$ENDIF defined(Win32) or defined(Win64) or defined(WinCE)}
  ;

function InGraphMode: Boolean;
begin
  Result := (PTCWrapperObject <> nil) and (PTCWrapperObject.IsOpen);
end;

var
  KeyBuffer: array[0..64] of Char;
  KeyBufHead, KeyBufTail: Integer;

function KeyBufEmpty: Boolean;
begin
  Result := KeyBufHead = KeyBufTail;
end;

procedure KeyBufAdd(Ch: Char);
begin
  { do nothing, if the buffer is full }
  if ((KeyBufTail + 1) = KeyBufHead) or
     ((KeyBufTail = High(KeyBuffer)) and (KeyBufHead = Low(KeyBuffer))) then
    exit;
  KeyBuffer[KeyBufTail] := Ch;
  Inc(KeyBufTail);
  if KeyBufTail > High(KeyBuffer) then
    KeyBufTail := Low(KeyBuffer);
end;

procedure KeyBufAdd(S: String);
var
  I: Integer;
begin
  for I := 1 to Length(S) do
    KeyBufAdd(S[I]);
end;

function KeyBufGet: Char;
begin
  if KeyBufHead <> KeyBufTail then
  begin
    Result := KeyBuffer[KeyBufHead];
    Inc(KeyBufHead);
    if KeyBufHead > High(KeyBuffer) then
      KeyBufHead := Low(KeyBuffer);
  end;
end;

procedure GetKeyEvents;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ev: IPTCEvent;
  KeyEv: IPTCKeyEvent;
begin
  repeat
    PTCWrapperObject.NextEvent(ev, False, [PTCKeyEvent, PTCCloseEvent]);
    if ev <> nil then
    begin
      case ev.EventType of
        PTCCloseEvent:
          begin
            { emulate Ctrl-C/Ctrl-Break, when the user
              presses the [X] button to close the window }
            KeyBufAdd(#3);
          end;
        PTCKeyEvent:
          begin
            KeyEv := ev as IPTCKeyEvent;
            if KeyEv.Press then
            begin
              if KeyEv.Alt then
              begin
                case KeyEv.Code of
                  PTCKEY_F1:     KeyBufAdd(#0#104);
                  PTCKEY_F2:     KeyBufAdd(#0#105);
                  PTCKEY_F3:     KeyBufAdd(#0#106);
                  PTCKEY_F4:     KeyBufAdd(#0#107);
                  PTCKEY_F5:     KeyBufAdd(#0#108);
                  PTCKEY_F6:     KeyBufAdd(#0#109);
                  PTCKEY_F7:     KeyBufAdd(#0#110);
                  PTCKEY_F8:     KeyBufAdd(#0#111);
                  PTCKEY_F9:     KeyBufAdd(#0#112);
                  PTCKEY_F10:    KeyBufAdd(#0#113);
                  PTCKEY_ONE:    KeyBufAdd(#0#120);
                  PTCKEY_TWO:    KeyBufAdd(#0#121);
                  PTCKEY_THREE:  KeyBufAdd(#0#122);
                  PTCKEY_FOUR:   KeyBufAdd(#0#123);
                  PTCKEY_FIVE:   KeyBufAdd(#0#124);
                  PTCKEY_SIX:    KeyBufAdd(#0#125);
                  PTCKEY_SEVEN:  KeyBufAdd(#0#126);
                  PTCKEY_EIGHT:  KeyBufAdd(#0#127);
                  PTCKEY_NINE:   KeyBufAdd(#0#128);
                  PTCKEY_ZERO:   KeyBufAdd(#0#129);
                  PTCKEY_MINUS:  KeyBufAdd(#0#130);
                  PTCKEY_EQUALS: KeyBufAdd(#0#131);
                  PTCKEY_Q:      KeyBufAdd(#0#16);
                  PTCKEY_W:      KeyBufAdd(#0#17);
                  PTCKEY_E:      KeyBufAdd(#0#18);
                  PTCKEY_R:      KeyBufAdd(#0#19);
                  PTCKEY_T:      KeyBufAdd(#0#20);
                  PTCKEY_Y:      KeyBufAdd(#0#21);
                  PTCKEY_U:      KeyBufAdd(#0#22);
                  PTCKEY_I:      KeyBufAdd(#0#23);
                  PTCKEY_O:      KeyBufAdd(#0#24);
                  PTCKEY_P:      KeyBufAdd(#0#25);
                  PTCKEY_A:      KeyBufAdd(#0#30);
                  PTCKEY_S:      KeyBufAdd(#0#31);
                  PTCKEY_D:      KeyBufAdd(#0#32);
                  PTCKEY_F:      KeyBufAdd(#0#33);
                  PTCKEY_G:      KeyBufAdd(#0#34);
                  PTCKEY_H:      KeyBufAdd(#0#35);
                  PTCKEY_J:      KeyBufAdd(#0#36);
                  PTCKEY_K:      KeyBufAdd(#0#37);
                  PTCKEY_L:      KeyBufAdd(#0#38);
                  PTCKEY_Z:      KeyBufAdd(#0#44);
                  PTCKEY_X:      KeyBufAdd(#0#45);
                  PTCKEY_C:      KeyBufAdd(#0#46);
                  PTCKEY_V:      KeyBufAdd(#0#47);
                  PTCKEY_B:      KeyBufAdd(#0#48);
                  PTCKEY_N:      KeyBufAdd(#0#49);
                  PTCKEY_M:      KeyBufAdd(#0#50);
                end;
              end
              else
                if KeyEv.Control then
                begin
                  case KeyEv.Code of
                    PTCKEY_ESCAPE:        KeyBufAdd(#27);
                    PTCKEY_F1:            KeyBufAdd(#0#94);
                    PTCKEY_F2:            KeyBufAdd(#0#95);
                    PTCKEY_F3:            KeyBufAdd(#0#96);
                    PTCKEY_F4:            KeyBufAdd(#0#97);
                    PTCKEY_F5:            KeyBufAdd(#0#98);
                    PTCKEY_F6:            KeyBufAdd(#0#99);
                    PTCKEY_F7:            KeyBufAdd(#0#100);
                    PTCKEY_F8:            KeyBufAdd(#0#101);
                    PTCKEY_F9:            KeyBufAdd(#0#102);
                    PTCKEY_F10:           KeyBufAdd(#0#103);
                    PTCKEY_TWO:           KeyBufAdd(#0#3);
                    PTCKEY_BACKSPACE:     KeyBufAdd(#127);
                    PTCKEY_A:             KeyBufAdd(#1);
                    PTCKEY_B:             KeyBufAdd(#2);
                    PTCKEY_C:             KeyBufAdd(#3);
                    PTCKEY_D:             KeyBufAdd(#4);
                    PTCKEY_E:             KeyBufAdd(#5);
                    PTCKEY_F:             KeyBufAdd(#6);
                    PTCKEY_G:             KeyBufAdd(#7);
                    PTCKEY_H:             KeyBufAdd(#8);
                    PTCKEY_I:             KeyBufAdd(#9);
                    PTCKEY_J:             KeyBufAdd(#10);
                    PTCKEY_K:             KeyBufAdd(#11);
                    PTCKEY_L:             KeyBufAdd(#12);
                    PTCKEY_M:             KeyBufAdd(#13);
                    PTCKEY_N:             KeyBufAdd(#14);
                    PTCKEY_O:             KeyBufAdd(#15);
                    PTCKEY_P:             KeyBufAdd(#16);
                    PTCKEY_Q:             KeyBufAdd(#17);
                    PTCKEY_R:             KeyBufAdd(#18);
                    PTCKEY_S:             KeyBufAdd(#19);
                    PTCKEY_T:             KeyBufAdd(#20);
                    PTCKEY_U:             KeyBufAdd(#21);
                    PTCKEY_V:             KeyBufAdd(#22);
                    PTCKEY_W:             KeyBufAdd(#23);
                    PTCKEY_X:             KeyBufAdd(#24);
                    PTCKEY_Y:             KeyBufAdd(#25);
                    PTCKEY_Z:             KeyBufAdd(#26);
                    PTCKEY_OPENBRACKET:   KeyBufAdd(#27);
                    PTCKEY_BACKSLASH:     KeyBufAdd(#28);
                    PTCKEY_CLOSEBRACKET:  KeyBufAdd(#29);
                    PTCKEY_SIX:           KeyBufAdd(#30);
                    PTCKEY_MINUS:         KeyBufAdd(#31);
                    PTCKEY_ENTER:         KeyBufAdd(#10);
                    PTCKEY_LEFT:          KeyBufAdd(#0#115);
                    PTCKEY_RIGHT:         KeyBufAdd(#0#116);
                    PTCKEY_HOME:          KeyBufAdd(#0#119);
                    PTCKEY_END:           KeyBufAdd(#0#117);
                    PTCKEY_PAGEUP:        KeyBufAdd(#0#132);
                    PTCKEY_PAGEDOWN:      KeyBufAdd(#0#118);
                  end;
                end
                else
                  if KeyEv.Shift then
                  begin
                    case KeyEv.Code of
                      PTCKEY_ESCAPE:    KeyBufAdd(#27);
                      PTCKEY_F1:        KeyBufAdd(#0#84);
                      PTCKEY_F2:        KeyBufAdd(#0#85);
                      PTCKEY_F3:        KeyBufAdd(#0#86);
                      PTCKEY_F4:        KeyBufAdd(#0#87);
                      PTCKEY_F5:        KeyBufAdd(#0#88);
                      PTCKEY_F6:        KeyBufAdd(#0#89);
                      PTCKEY_F7:        KeyBufAdd(#0#90);
                      PTCKEY_F8:        KeyBufAdd(#0#91);
                      PTCKEY_F9:        KeyBufAdd(#0#92);
                      PTCKEY_F10:       KeyBufAdd(#0#93);
                      PTCKEY_BACKSPACE: KeyBufAdd(#8);
                      PTCKEY_TAB:       KeyBufAdd(#0#15);
                      PTCKEY_ENTER:     KeyBufAdd(#13);
                      PTCKEY_INSERT:    KeyBufAdd(#0#82);
                      PTCKEY_DELETE:    KeyBufAdd(#0#83);
                      PTCKEY_LEFT:      KeyBufAdd(#0#75);
                      PTCKEY_UP:        KeyBufAdd(#0#72);
                      PTCKEY_RIGHT:     KeyBufAdd(#0#77);
                      PTCKEY_DOWN:      KeyBufAdd(#0#80);
                      PTCKEY_HOME:      KeyBufAdd(#0#71);
                      PTCKEY_END:       KeyBufAdd(#0#79);
                      PTCKEY_PAGEUP:    KeyBufAdd(#0#73);
                      PTCKEY_PAGEDOWN:  KeyBufAdd(#0#81);
                      else
                        if (KeyEv.Unicode >= 32) and (KeyEv.Unicode <= 127) then
                          KeyBufAdd(Chr(KeyEv.Unicode));
                    end;
                  end
                  else
                  begin
                    case KeyEv.Code of
                      PTCKEY_ESCAPE:    KeyBufAdd(#27);
                      PTCKEY_F1:        KeyBufAdd(#0#59);
                      PTCKEY_F2:        KeyBufAdd(#0#60);
                      PTCKEY_F3:        KeyBufAdd(#0#61);
                      PTCKEY_F4:        KeyBufAdd(#0#62);
                      PTCKEY_F5:        KeyBufAdd(#0#63);
                      PTCKEY_F6:        KeyBufAdd(#0#64);
                      PTCKEY_F7:        KeyBufAdd(#0#65);
                      PTCKEY_F8:        KeyBufAdd(#0#66);
                      PTCKEY_F9:        KeyBufAdd(#0#67);
                      PTCKEY_F10:       KeyBufAdd(#0#68);
                      PTCKEY_BACKSPACE: KeyBufAdd(#8);
                      PTCKEY_TAB:       KeyBufAdd(#9);
                      PTCKEY_ENTER:     KeyBufAdd(#13);
                      PTCKEY_INSERT:    KeyBufAdd(#0#82);
                      PTCKEY_DELETE:    KeyBufAdd(#0#83);
                      PTCKEY_LEFT:      KeyBufAdd(#0#75);
                      PTCKEY_UP:        KeyBufAdd(#0#72);
                      PTCKEY_RIGHT:     KeyBufAdd(#0#77);
                      PTCKEY_DOWN:      KeyBufAdd(#0#80);
                      PTCKEY_HOME:      KeyBufAdd(#0#71);
                      PTCKEY_END:       KeyBufAdd(#0#79);
                      PTCKEY_PAGEUP:    KeyBufAdd(#0#73);
                      PTCKEY_PAGEDOWN:  KeyBufAdd(#0#81);
                      else
                        if (KeyEv.Unicode >= 32) and (KeyEv.Unicode <= 127) then
                          KeyBufAdd(Chr(KeyEv.Unicode));
                  end;
              end;
            end;
          end;
      end;
    end;
  until ev = nil;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ev: TPTCEvent;
  KeyEv: TPTCKeyEvent;
begin
  ev := nil;
  try
    repeat
      PTCWrapperObject.NextEvent(ev, False, [PTCKeyEvent]);
      if ev <> nil then
      begin
        KeyEv := TPTCKeyEvent(ev);
        if KeyEv.Press then
        begin
          case KeyEv.Code of
            PTCKEY_BACKSPACE:
              if KeyEv.Control then
                KeyBufAdd(#127)
              else
                KeyBufAdd(#8);
            PTCKEY_ENTER:  KeyBufAdd(#13);
            PTCKEY_ESCAPE: KeyBufAdd(#27);
            PTCKEY_INSERT: KeyBufAdd(#0#82);
            PTCKEY_DELETE: KeyBufAdd(#0#83);
            PTCKEY_LEFT:   KeyBufAdd(#0#75);
            PTCKEY_UP:     KeyBufAdd(#0#72);
            PTCKEY_RIGHT:  KeyBufAdd(#0#77);
            PTCKEY_DOWN:   KeyBufAdd(#0#80);
            PTCKEY_HOME:     KeyBufAdd(#0#71);
            PTCKEY_END:      KeyBufAdd(#0#79);
            PTCKEY_PAGEUP:   KeyBufAdd(#0#73);
            PTCKEY_PAGEDOWN: KeyBufAdd(#0#81);
            PTCKEY_F1:     KeyBufAdd(#0#59);
            PTCKEY_F2:     KeyBufAdd(#0#60);
            PTCKEY_F3:     KeyBufAdd(#0#61);
            PTCKEY_F4:     KeyBufAdd(#0#62);
            PTCKEY_F5:     KeyBufAdd(#0#63);
            PTCKEY_F6:     KeyBufAdd(#0#64);
            PTCKEY_F7:     KeyBufAdd(#0#65);
            PTCKEY_F8:     KeyBufAdd(#0#66);
            PTCKEY_F9:     KeyBufAdd(#0#67);
            PTCKEY_F10:    KeyBufAdd(#0#68);
            else
              if (KeyEv.Unicode >= 32) and (KeyEv.Unicode <= 127) then
                KeyBufAdd(Chr(KeyEv.Unicode));
          end;
        end;
      end;
    until ev = nil;
  finally
    ev.Free;
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

function KeyPressed: Boolean;
begin
  if not InGraphMode then
  begin
{$IFDEF HasCRT}
    Result := crt.KeyPressed
{$ELSE HasCRT}
    Result := False;
{$ENDIF HasCRT}
  end
  else
  begin
    GetKeyEvents;
    Result := not KeyBufEmpty;
  end;
end;

function ReadKey: Char;
{$IFDEF UNIX}
var
  req, rem: TTimeSpec;
{$ENDIF UNIX}
begin
  if not InGraphMode then
  begin
{$IFDEF HasCRT}
    Result := crt.ReadKey;
{$ELSE HasCRT}
    Result := #0;
{$ENDIF HasCRT}
  end
  else
  begin
    while not KeyPressed do
    begin
{$IFDEF UNIX}
      req.tv_sec := 0;
      req.tv_nsec := 1000000;
      fpnanosleep(@req, @rem);
{$ENDIF UNIX}
{$IF defined(Win32) or defined(Win64) or defined(WinCE)}
      Sleep(1);
{$ENDIF defined(Win32) or defined(Win64) or defined(WinCE)}
    end;
    Result := KeyBufGet;
  end;
end;

procedure ClrScr;
begin
{$IFDEF HasCRT}
  crt.ClrScr;
{$ENDIF HasCRT}
end;

procedure ClrEol;
begin
{$IFDEF HasCRT}
  crt.ClrEol;
{$ENDIF HasCRT}
end;

procedure GotoXY(X, Y: tcrtcoord);
begin
{$IFDEF HasCRT}
  crt.GotoXY(X, Y);
{$ENDIF HasCRT}
end;

procedure TextColor(Color: Byte);
begin
{$IFDEF HasCRT}
  crt.TextColor(Color);
{$ENDIF HasCRT}
end;

procedure TextBackground(Color: Byte);
begin
{$IFDEF HasCRT}
  crt.TextBackground(Color);
{$ENDIF HasCRT}
end;

procedure Delay(MS: Word);
begin
{$IFDEF HasCRT}
  crt.Delay(MS);
{$ENDIF HasCRT}
end;

procedure Sound(HZ: Word);
begin
{$IFDEF HasCRT}
  crt.Sound(HZ);
{$ENDIF HasCRT}
end;

procedure NoSound;
begin
{$IFDEF HasCRT}
  crt.NoSound;
{$ENDIF HasCRT}
end;

end.
