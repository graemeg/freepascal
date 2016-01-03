PROGRAM Sterne;


uses Exec, Graphics, Intuition, Utility, systemvartags;



CONST   MAX_STERNE = 42;
        MAX_GESCHW = 15;

TYPE    Star = packed Record
                 x,y :Integer;
                 msin :Real;
                 mcos :Real;
                 d   :Integer;
                 v   :Integer;
               End;

VAR     Scr     :pScreen;
        Win     :pWindow;
        Msg     :pIntuiMessage;
        Ende    :Boolean;
        Stars   :Array[1..MAX_STERNE] of Star;
        factor  :Real;
        col     :Integer;
        dum     :Longint;


PROCEDURE newStern(num :Integer);

BEGIN
  col:=Random(360);
  Stars[num].x := Scr^.Width shr 1;
  Stars[num].y := Scr^.Height shr 1;
  Stars[num].msin := sin(col*factor);
  Stars[num].mcos := cos(col*factor);
  Stars[num].d := 0;
  Stars[num].v := Random(MAX_GESCHW)+2;
END;


PROCEDURE moveStern(num :Integer);

BEGIN
  Stars[num].d:=Stars[num].d+Stars[num].v;
  Stars[num].x:=Round(Stars[num].d*Stars[num].msin)+Scr^.Width shr 1;
  Stars[num].y:=Round(Stars[num].d*Stars[num].mcos)+Scr^.Height shr 1;
  {Inc(Stars[num].v);}
END;


PROCEDURE drawSterne;

BEGIN
  For dum:=1 to MAX_STERNE Do Begin
    If Stars[dum].v=0 Then Begin
      If Random(10)>4 Then
        newStern(dum);
    End Else If Stars[dum].d>Scr^.Width shr 1 Then Begin
      SetAPen(Win^.RPort,0);
      If WritePixel(Win^.RPort,(Stars[dum].x),(Stars[dum].y))=0 Then;
      Stars[dum].v:=0;
    End Else Begin
      SetAPen(Win^.RPort,0);
      If WritePixel(Win^.RPort,(Stars[dum].x),(Stars[dum].y))=0 Then;
      moveStern(dum);
      col:=(Stars[dum].d shl 5) Div Scr^.Height shr 1;
      If col>7 Then
        col:=7;
      SetAPen(Win^.RPort,col);
      If WritePixel(Win^.RPort,(Stars[dum].x),(Stars[dum].y))=0 Then;
    End;
  End;
END;


PROCEDURE initSterne;

BEGIN
  For dum:=1 to MAX_STERNE Do begin
    Stars[dum].x := Scr^.Width shr 1;
    Stars[dum].y := Scr^.Height shr 1;
    Stars[dum].msin := 0.0;
    Stars[dum].mcos := 0.0;
    Stars[dum].d := 0;
    Stars[dum].v := 0;
  end;
  factor:=PI/180;
END;


PROCEDURE CleanUp(str:string; code : Longint);

BEGIN
  If assigned(Win) Then
    CloseWindow(Win);
  If assigned(Scr) then CloseScreen(Scr);
  if str <> '' then writeln(str);
  Halt(code);
END;

PROCEDURE Init;

BEGIN

  Scr:=Nil;  Win:=Nil;

  Scr := OpenScreenTags(NIL,[
                   SA_Depth,     3,
                   SA_DisplayID, HIRES_KEY,
                   TAG_END]);

  If Scr=Nil Then CleanUp('No screen',20);

  Win:=OpenWindowTags(Nil, [
                        WA_Flags, WFLG_BORDERLESS,
                        WA_IDCMP, IDCMP_MOUSEBUTTONS,
                        WA_CustomScreen, Scr,
                        TAG_DONE]);

  If Win=Nil Then CleanUp('No window',20);

  initSterne;

  SetRGB4(@Scr^.ViewPort, 0, $0,$0,$0);
  SetRGB4(@Scr^.ViewPort, 1, $3,$3,$3);
  SetRGB4(@Scr^.ViewPort, 2, $6,$6,$6);
  SetRGB4(@Scr^.ViewPort, 3, $b,$b,$b);
  SetRGB4(@Scr^.ViewPort, 4, $c,$c,$c);
  SetRGB4(@Scr^.ViewPort, 5, $d,$d,$d);
  SetRGB4(@Scr^.ViewPort, 6, $e,$e,$e);
  SetRGB4(@Scr^.ViewPort, 7, $f,$f,$f);

END;



BEGIN
  Init;
  Ende:=false;
  Repeat
    drawSterne;
    Msg:=pIntuiMessage(GetMsg(Win^.UserPort));
    If Msg<>Nil Then Begin
      ReplyMsg(Pointer(Msg));
      Ende:=true;
    End;
  Until Ende;
  CleanUp('',0);
END.
