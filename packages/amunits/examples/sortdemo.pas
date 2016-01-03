PROGRAM SortDemo;

{ Graphical demonstration of sorting algorithms (W. N~ker, 02/96) }
{ based on "Sortieren" of Purity #48 }

{
    Translated to PCQ from Kick(Maxon) Pascal.
    Updated the source to 2.0+.
    Now uses GadTools for menus.
    Added CloseWindowSafely.
    Cleaned up the menuhandling.
    Added LockWinSize and RestoreWin, now the
    window will be locked on showtime.

    The German text was translated to English
    by Andreas Neumann, thanks Andreas.
    Jun 03 1998.

    Translated to FPC Pascal.
    Removed CloseWindowSafely, have do add
    that procedure to Intuition.
    Fixed a bug, when you halt the show the
    window stayed locked.
    Aug 23 1998.

    Added MessageBox for report.
    31 Jul 2000.

    Removed opening of graphics.library.
    21 Mar 2001.

    Reworked to use systemvartags.
    28 Nov 2002.

    nils.sjoholm@mailbox.swipnet.se

    One last remark, the heapsort can't be stoped
    so you have to wait until it's finished.
}

uses Exec, Intuition, Graphics, Utility, GadTools, msgbox,systemvartags;


CONST
      vers : string = '$VER: SortDemo 1.3 ' + {$I %DATE%} + ' ' + {$I %TIME%}#0;

      nmax=2000;

      MinWinX = 80;
      MinWiny = 80;

      w         : pWindow  = Nil;
      s         : pScreen  = Nil;
      MenuStrip : pMenu    = Nil;
      vi        : Pointer  = Nil;


      modenames : Array[0..7] of string[10] = (
                                'Heapsort',
                                'Shellsort',
                                'Pick out',
                                'Insert',
                                'Shakersort',
                                'Bubblesort',
                                'Quicksort',
                                'Mergesort');

      { The easiest way to use gadtoolsmenus in FPC is
        to have them as const types. No need to cast
        strings to PChar. That we have to use recordmembers
        name is a pain.
      }

      nm : array[0..21] of tNewMenu = (
      (nm_Type: NM_TITLE; nm_Label: 'Demo';nm_CommKey: NIL; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'Start';nm_CommKey: 'S'; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'Stop';nm_CommKey: 'H'; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),

      { this will be a barlabel, have to set this one later }
      (nm_Type: NM_ITEM;  nm_Label: NIL; nm_CommKey: NIL; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),

      (nm_Type: NM_ITEM;  nm_Label: 'Quit';  nm_CommKey: 'Q'; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),
      (nm_Type: NM_TITLE; nm_Label: 'Algorithm'; nm_CommKey: NIL; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'HeapSort'; nm_CommKey: '1'; nm_Flags:
       CHECKIT+CHECKED+MENUTOGGLE; nm_MutualExclude: 254; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'ShellSort'; nm_CommKey: '2'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 253; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'Pick out'; nm_CommKey: '3'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 251; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'Insert'; nm_CommKey: '4'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 247; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'ShakerSort'; nm_CommKey: '5'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 239; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'BubbleSort'; nm_CommKey: '6'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 223; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'QuickSort'; nm_CommKey: '7'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 191; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'MergeSort'; nm_CommKey: '8'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 127; nm_UserData: NIL),
      (nm_Type: NM_TITLE; nm_Label: 'Preferences'; nm_CommKey: NIL; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'Data'; nm_CommKey: NIL; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),
      (nm_Type: NM_SUB;   nm_Label: 'Random'; nm_CommKey: 'R'; nm_Flags:
       CHECKIT+CHECKED+MENUTOGGLE; nm_MutualExclude: 2; nm_UserData: NIL),
      (nm_Type: NM_SUB;   nm_Label: 'Malicious'; nm_CommKey: 'M'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 1; nm_UserData: NIL),
      (nm_Type: NM_ITEM;  nm_Label: 'Diagram'; nm_CommKey: NIL; nm_Flags: 0;
       nm_MutualExclude: 0; nm_UserData: NIL),
      (nm_Type: NM_SUB;   nm_Label: 'Needles'; nm_CommKey: 'N'; nm_Flags:
       CHECKIT+CHECKED+MENUTOGGLE; nm_MutualExclude: 2; nm_UserData: NIL),
      (nm_Type: NM_SUB;   nm_Label: 'Dots'; nm_CommKey: 'D'; nm_Flags:
       CHECKIT+MENUTOGGLE; nm_MutualExclude: 1; nm_UserData: NIL),
      (nm_Type: NM_END;   nm_Label: NIL; nm_CommKey: NIL; nm_Flags:
       0;nm_MutualExclude:0;nm_UserData:NIL));


VAR sort: ARRAY[1..nmax] OF Real;
    sort2: ARRAY[1..nmax] OF Real;  { for dumb Mergesort %-( }
    num,range,modus : Integer;
    rndom,needles   : Boolean;
    Rast            : pRastPort;
    QuitStopDie     : Boolean;
    Msg             : pMessage;
    wintitle        : string[80];
    scrtitle        : string[80];

Procedure CleanUp(s : string; err : Integer);
begin
    if assigned(MenuStrip) then begin
       ClearMenuStrip(w);
       FreeMenus(MenuStrip);
    end;
    if assigned(vi) then FreeVisualInfo(vi);
    if assigned(w) then CloseWindow(w);
    if s <> '' then MessageBox('SortDemo Report',s,'OK');
    Halt(err);
end;

Procedure RestoreWin;
var
   dummy : Boolean;
begin
   dummy := WindowLimits(w,MinWinX,MinWinY,-1,-1);
end;

Procedure LockWinSize(x,y,x2,y2 : Integer);
var
   dummy : Boolean;
begin
   dummy := WindowLimits(w,x,y,x2,y2);
end;

FUNCTION cancel: Boolean;
{ checked while sorting }
VAR m,i,s: Integer;
    result : boolean;
    IM : pIntuiMessage;
BEGIN
  result := False;
  IM := pIntuiMessage(GetMsg(w^.UserPort));
  IF IM<>Nil THEN BEGIN
    IF IM^.IClass=IDCMP_CLOSEWINDOW THEN
      result := True;   { Close-Gadget }
    IF IM^.IClass=IDCMP_MENUPICK THEN BEGIN
      m := IM^.Code AND $1F;
      i := (IM^.Code SHR 5) AND $3F;
      s := (IM^.Code SHR 11) AND $1F;
      IF (m=0) AND (i=1) THEN  result := True;  { Menu item "Stop" }
    END;
    ReplyMsg(pMessage(Msg));
  END;
  cancel := result;
END;


PROCEDURE showstack(size: Integer);
{ little diagram showing the depth of Quicksort's recursion :-) }
BEGIN
  SetAPen(Rast,2); IF size>0 THEN RectFill(Rast,0,0,3,size-1);
  SetAPen(Rast,0); RectFill(Rast,0,size,3,size);
END;


PROCEDURE setpixel(i: Integer);
BEGIN
  SetAPen(Rast,1);
  IF needles THEN BEGIN
    Move(Rast,i,range); Draw(Rast,i,Round((1-sort[i])*range));
  END ELSE
    IF WritePixel(Rast,i,Round((1-sort[i])*range))=0 THEN;
END;

PROCEDURE clearpixel(i: Integer);
BEGIN
  SetAPen(Rast,0);
  IF needles THEN BEGIN
    Move(Rast,i,range); Draw(Rast,i,Round((1-sort[i])*range));
  END ELSE
    IF WritePixel(Rast,i,Round((1-sort[i])*range))=0 THEN;
END;

procedure Exchange(var first,second : real);
var
  temp : real;
begin
  temp := first;
  first := second;
  second := temp;
end;

PROCEDURE swapit(i,j: integer);
BEGIN
  clearpixel(i); clearpixel(j);
  Exchange(sort[i],sort[j]);
  setpixel(i); setpixel(j);
END;

FUNCTION descending(i,j: Integer): Boolean;
BEGIN
  descending := sort[i]>sort[j];
END;

Function IntToStr (I : Longint) : String;

     Var S : String;

     begin
      Str (I,S);
      IntToStr:=S;
     end;


PROCEDURE settitles(time: Longint);
VAR
  s : string[80];
BEGIN
  s := modenames[modus];
  IF time=0 THEN
    wintitle := s + ' running ...'
  ELSE IF time < 0 then
    wintitle := '<- ' + IntToStr(num) + ' Data ->'
  ELSE
    wintitle := IntToStr(time) + ' Seconds';
  scrtitle := strpas(@vers[6]) + ' - ' + s;
  wintitle := wintitle + #0;
  scrtitle := scrtitle + #0;
  SetWindowTitles(w,@wintitle[1],@scrtitle[1]);
END;

PROCEDURE refresh;
{ react on new size of window/init data }
VAR i: Integer;
BEGIN
  num := w^.GZZWidth; IF num>nmax THEN num := nmax;
  range := w^.GZZHeight;
  settitles(-1);
  SetRast(Rast,0);    { clear screen }
  FOR i := 1 TO num DO BEGIN
    IF rndom THEN sort[i] := Random  { produces 0..1 }
      ELSE sort[i] := (num-i)/num;
    setpixel(i);
  END;
END;

{ *#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* }
{ *#*#*#*#*#*#*#*#*#*#*# The sorting algorithms! #*#*#*#*#*#*#*#*#*#*#*#* }
{ *#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* }

PROCEDURE bubblesort;
{ like the head of a beer, reaaal slow and easy-going }
VAR i,j,max: Integer;
BEGIN
  LockWinSize(w^.Width,w^.Height,w^.Width,w^.Height);
  max := num;
  REPEAT
    j := 1;
    FOR i := 1 TO max-1 DO
      IF descending(i,i+1) THEN BEGIN
        swapit(i,i+1); j := i;
      END;
    max := j;
  UNTIL (max=1) OR cancel;
  RestoreWin;
END;

PROCEDURE shakersort;
{ interesting variant, but bubblesort still remains hopelessness }
{ (because it only compares and swaps immediate adjacent units)  }
VAR i,j,min,max: Integer;
BEGIN
  LockWinSize(w^.Width,w^.Height,w^.Width,w^.Height);
  min := 1;
  max := num;
  REPEAT
    j := min;
    FOR i := min TO max-1 DO
      IF descending(i,i+1) THEN BEGIN
        swapit(i,i+1); j := i;
      END;
    max := j;
    j := max;
    FOR i := max DOWNTO min+1 DO
      IF descending(i-1,i) THEN BEGIN
        swapit(i,i-1); j := i;
      END;
    min := j;
  UNTIL (max=min) OR cancel;
  RestoreWin;
END;

PROCEDURE e_sort;
{ Insert: a pretty human strategy }
VAR i,j: Integer;
BEGIN
  LockWinSize(w^.Width,w^.Height,w^.Width,w^.Height);
  FOR i := 2 TO num DO BEGIN
    j := i;
    WHILE j>1 DO
      IF descending(j-1,j) THEN BEGIN
        swapit(j-1,j); Dec(j);
      END ELSE
        j := 1;
    IF cancel THEN begin
        RestoreWin;
        Exit;
    end;
  END;
  RestoreWin;
END;

PROCEDURE a_sort;
{ Pick out: Preparation is one half of a life }
{ Take a look at the ridiculous low percentage of successful comparisions:  }
{ Although there are only n swaps, there are n^2/2 comparisions!            }
{ Both is a record, one in a good sense, the other one in a bad sense.      }

VAR i,j,minpos: Integer;
    min: Real;
BEGIN
  LockWinSize(w^.Width,w^.Height,w^.Width,w^.Height);
  FOR i := 1 TO num-1 DO BEGIN
    minpos := i; min := sort[i];
    FOR j := i+1 TO num DO
      IF descending(minpos,j) THEN
        minpos := j;
    IF minpos<>i THEN swapit(i,minpos);
    IF cancel THEN begin
        RestoreWin;
        Exit;
    end;
  END;
  RestoreWin;
END;

PROCEDURE shellsort;
{ brilliant extension of E-Sort, stunning improvement of efficience }
VAR i,j,gap: Integer;
BEGIN
  LockWinSize(w^.Width,w^.Height,w^.Width,w^.Height);
  gap := num DIV 2;
  REPEAT
    FOR i := 1+gap TO num DO BEGIN
      j := i;
      WHILE j>gap DO
        IF descending(j-gap,j) THEN BEGIN
          swapit(j,j-gap); j := j-gap;
        END ELSE
          j := 1;
      IF cancel THEN begin
          RestoreWin;
          Exit;
      end;
    END;
    gap := gap DIV 2;
  UNTIL gap=0;
  RestoreWin;
END;

PROCEDURE seepaway(i,max: Integer);
{ belongs to heapsort }
VAR j: Integer;
BEGIN
  j := 2*i;
  WHILE j<=max DO BEGIN
    IF j<max THEN IF descending(j+1,j) THEN
      Inc(j);
    IF descending(j,i) THEN BEGIN
      swapit(j,i);
      i := j; j := 2*i;
    END ELSE
      j := max+1; { cancels }
  END;
END;

PROCEDURE heapsort;
{ this genius rules over the chaos: it's the best algorithm, I know about    }
{ The only disadvantage compared with shellsort: it's not easy to understand }
{ and impossible to know it by heart. }
VAR i,j: Integer;
BEGIN
  LockWinSize(w^.Width,w^.Height,w^.Width,w^.Height);
  i := num DIV 2 + 1;
  j := num;
  WHILE i>1 DO BEGIN
    Dec(i); seepaway(i,j);
  END;
  WHILE j>1 DO BEGIN
    swapit(i,j);
    Dec(j); seepaway(i,j);
  END;
  RestoreWin;
END;

PROCEDURE quicksort;
{ "divide and rule": a classic, but recursive  >>-( }
{ In this demonstration it is faster than heapsort, but does considerable }
{ more unsuccessful comparisions. }
VAR stack: ARRAY[1..100] OF RECORD li,re: Integer; END;
    sp,l,r,m,i,j: Integer;
BEGIN
  LockWinSize(w^.Width,w^.Height,w^.Width,w^.Height);
  sp := 1; stack[1].li := 1; stack[1].re := num;
  REPEAT
    l := stack[sp].li; r := stack[sp].re; Dec(sp);
    showstack(sp);
    m := (l+r) DIV 2;
    i := l; j := r;
    REPEAT
      WHILE descending(m,i) DO Inc(i);
      WHILE descending(j,m) DO Dec(j);
      IF j>i THEN swapit(i,j);
      IF m=i THEN m := j ELSE IF m=j THEN m := i; { ahem ... }
      { This "Following" of the reference data is only required because  }
      { I stubborn call the comparision function, and this one only gets }
      { indices on the values which have to be compared. }
    UNTIL i>=j;
    IF i>l THEN BEGIN
      Inc(sp); stack[sp].li := l; stack[sp].re := i; END;
    IF i+1<r THEN BEGIN
      Inc(sp); stack[sp].li := i+1; stack[sp].re := r; END;
  UNTIL (sp=0) OR cancel;
  RestoreWin;
END;

PROCEDURE mergesort;
{ *the* algorithm for lists with pointers on it, for arrays rather }
{ inacceptable. The non.recursive implementation came out pretty more }
{ complicated than the one for quicksort, as quicksort first does }
{ something and then recurses; with mergesort it is the other way round. }
VAR stack: ARRAY[1..100] OF RECORD li,re,mi: Integer; END;
    sp,l,r,i,j,k,m: Integer;
BEGIN
  LockWinSize(w^.Width,w^.Height,w^.Width,w^.Height);
  sp := 1; stack[1].li := 1; stack[1].re := num; stack[1].mi := 0;
  REPEAT
    l := stack[sp].li; r := stack[sp].re; m := stack[sp].mi; Dec(sp);
    showstack(sp);
    IF m>0 THEN BEGIN { put two halfs together }
      { Unfortunately it is only possible in an efficient way by using }
      { extra memory; mergesort really is something for lists with }
      { pointers originally ... }
      FOR i := m DOWNTO l do sort2[i] := sort[i];  i := l;
      FOR j := m+1 TO r DO sort2[r+m+1-j] := sort[j];  j := r;
      FOR k := l TO r DO BEGIN
        clearpixel(k);
        IF sort2[i]<sort2[j] THEN BEGIN
          sort[k] := sort2[i]; Inc(i);
        END ELSE BEGIN
          sort[k] := sort2[j]; Dec(j);
        END;
        setpixel(k);
      END;
    END ELSE IF l<r THEN BEGIN
      { create two halfs and the order to put them together }
      m := (l+r) DIV 2;
      Inc(sp); stack[sp].li := l; stack[sp].mi := m; stack[sp].re := r;
      Inc(sp); stack[sp].li := m+1; stack[sp].mi := 0; stack[sp].re := r;
      Inc(sp); stack[sp].li := l; stack[sp].mi := 0; stack[sp].re := m;
    END;
  UNTIL (sp=0) OR cancel;
  RestoreWin;
END;


Procedure OpenEverything;
begin

    s := LockPubScreen(nil);
    if s = nil then CleanUp('Could not lock pubscreen',10);

    vi := GetVisualInfoA(s, NIL);
    if vi = nil then CleanUp('No visual info',10);

    w := OpenWindowTags(NIL, [
                WA_IDCMP,         IDCMP_CLOSEWINDOW or IDCMP_MENUPICK or
IDCMP_NEWSIZE,
                WA_Left,          0,
                WA_Top,           s^.BarHeight+1,
                WA_Width,         224,
                WA_Height,        s^.Height-(s^.BarHeight-1),
                WA_MinWidth,      MinWinX,
                WA_MinHeight,     MinWinY,
                WA_MaxWidth,      -1,
                WA_MaxHeight,     -1,
                WA_DepthGadget,   ltrue,
                WA_DragBar,       ltrue,
                WA_CloseGadget,   ltrue,
                WA_SizeGadget,    ltrue,
                WA_Activate,      ltrue,
                WA_SizeBRight,    ltrue,
                WA_GimmeZeroZero, ltrue,
                WA_PubScreen,     s,
                TAG_END]);

    IF w=NIL THEN CleanUp('Could not open window',20);

    Rast := w^.RPort;

    { Here we set the barlabel }
    nm[3].nm_Label := PChar(NM_BARLABEL);

    if pExecBase(_ExecBase)^.LibNode.Lib_Version >= 39 then begin
        MenuStrip := CreateMenus(@nm,[
                     GTMN_FrontPen, 1, TAG_END]);
    end else MenuStrip := CreateMenusA(@nm,NIL);

    if MenuStrip = nil then CleanUp('Could not open Menus',10);
    if LayoutMenusA(MenuStrip,vi,NIL)=false then
        CleanUp('Could not layout Menus',10);

    if SetMenuStrip(w, MenuStrip) = false then
        CleanUp('Could not set the Menus',10);

end;

PROCEDURE ProcessIDCMP;
VAR
    IMessage    : tIntuiMessage;
    IPtr    : pIntuiMessage;

    Procedure ProcessMenu;
    var
    MenuNumber  : Integer;
    ItemNumber  : Integer;
    SubItemNumber   : Integer;
    t0,t1,l         : Longint;

    begin
    if IMessage.Code = MENUNULL then
        Exit;

    MenuNumber := MenuNum(IMessage.Code);
    ItemNumber := ItemNum(IMessage.Code);
    SubItemNumber := SubNum(IMessage.Code);

    case MenuNumber of
      0 : begin
          case ItemNumber of
             0 : begin
                   refresh;
                   settitles(0);
                   CurrentTime(t0,l);
                   CASE modus OF
                     0: heapsort;
                     1: shellsort;
                     2: a_sort;
                     3: e_sort;
                     4: shakersort;
                     5: bubblesort;
                     6: quicksort;
                     7: mergesort;
                   END;
                   CurrentTime(t1,l);
                   settitles(t1-t0);
                 end;
             3 : QuitStopDie := True;
          end;
          end;
      1 : begin
          case ItemNumber of
              0..7 : modus := ItemNumber;
          end;
          settitles(-1);
          end;
      2 : begin
          case ItemNumber of
             0 : begin
                 case SubItemNumber of
                    0 : if not rndom then rndom := true;
                    1 : if rndom then rndom := false;
                 end;
                 end;
             1 : begin
                 case SubItemNumber of
                    0 : if not needles then needles := true;
                    1 : if needles then needles := false;
                 end;
                 end;
          end;
          end;
    end;
    end;

begin
    IPtr := pIntuiMessage(Msg);
    IMessage := IPtr^;
    ReplyMsg(Msg);

    case IMessage.IClass of
      IDCMP_MENUPICK    : ProcessMenu;
      IDCMP_NEWSIZE     : refresh;
      IDCMP_CLOSEWINDOW : QuitStopDie := True;
    end;
end;



begin
   OpenEverything;
   QuitStopDie := False;
   modus := 0;
   needles := true;
   rndom := true;
   refresh;
   repeat
   Msg := WaitPort(w^.UserPort);
   Msg := GetMsg(w^.UserPort);
       ProcessIDCMP;
   until QuitStopDie;
   CleanUp('',0);
end.
