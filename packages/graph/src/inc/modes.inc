  {

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    This include implements video mode management.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{-----------------------------------------------------------------------}
{                          Internal routines                            }
{-----------------------------------------------------------------------}

 procedure res2Mode(x, y, maxColor: longint; var driver,mode: smallInt);
 var
   l: longint;
 begin
   case maxColor of
     2: driver := D1bit;
     4: driver := D2bit;
     16: driver := D4bit;
     64: driver := D6bit;
     256: driver := D8bit;
     4096: driver := D12bit;
     32768: driver := D15bit;
     65536: driver := D16bit;
{    not yet supported
     65536*256: driver := D24bit;
     65536*65536: driver := D32bit;}
     else
       begin
         driver := maxsmallint;
         exit;
       end;
   end;
   { Check whether this is known/predefined mode }
   for l := lowNewMode to highNewMode do
     if (resolutions[l].x = x) and
        (resolutions[l].y = y) then
       begin
         { Found! }
         mode := l;
         exit;
       end;
   { Not Found }
   mode := maxsmallint;
 end;

function mode2res(modeNr: smallInt; var x,y: longint): boolean;
begin
  if (modeNr < lowNewMode) or
     (modeNr > highNewMode) then
    begin
      mode2res := false;
      exit;
    end;
  mode2res := true;
  x := resolutions[modeNr].x;
  y := resolutions[modeNr].y;
end;


 procedure addmode(const mode: TModeInfo);
  {********************************************************}
  { Procedure AddMode()                                    }
  {--------------------------------------------------------}
  { This routine adds <mode> to the list of recognized     }
  { modes. Duplicates are allowed.                         }
  {********************************************************}
  var
    i,driverNr, modeNr: smallint;
<<<<<<< HEAD
<<<<<<< HEAD
    prev: PNewModeList;
    list: PModeInfo;
    newMode: PModeInfo;
    newList: PNewModeList;
    newLst: PNewModeList;
  begin
    { TP-like mode stuff }
    if not assigned(ModeList) then
      begin
        new(ModeList);
        System.move(mode, ModeList^, sizeof(Mode));
        newMode := ModeList;
      end
    else
      begin
        list := ModeList;
        { go to the end of the list }
        while assigned(list^.next) do
          list:=list^.next;
        new(newMode);
        list^.next := newMode;
        System.move(mode, newMode^, sizeof(Mode));
      end;

=======
=======
>>>>>>> origin/fixes_2_2
    prev: PModeInfo;
    list: PModeInfo;
    newlst : PModeInfo;
  begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    res2Mode(mode.maxx+1,mode.maxy+1,mode.maxColor,driverNr,ModeNr);
    { bitdepth supported? }
    if (driverNr <> maxsmallint) then
     begin
      { Yes, add the mode }
      if not assigned(newModeList.modeinfo[driverNr]) then
        begin
{$ifdef logging}
          logln('Adding resolution '+strf(modenr)+' for drivernr '+strf(drivernr)+
                ' ('+strf(mode.maxx)+'x'+strf(mode.maxy)+')');
{$endif logging}
          new(newModeList.modeinfo[driverNr]);
<<<<<<< HEAD
<<<<<<< HEAD
          newModeList.modeinfo[driverNr]^.Mode := newMode;
=======
          newModeList.modeinfo[driverNr]^ := mode;
>>>>>>> graemeg/fixes_2_2
=======
          newModeList.modeinfo[driverNr]^ := mode;
>>>>>>> origin/fixes_2_2
          newModeList.modeinfo[driverNr]^.next:=nil;
        end
      else
        begin
          prev := nil;
<<<<<<< HEAD
<<<<<<< HEAD
          newList := newModeList.modeinfo[driverNr];
          { sort first by x resolution, then by yresolution }
          while assigned(newList) and
                ((newList^.Mode^.maxx < mode.maxx) or
                 ((newList^.Mode^.maxx = mode.maxx) and
                  (newList^.Mode^.maxy < mode.maxy))) do
            begin
              prev := newList;
              newList := newList^.next;
            end;
          { mode already exists? -> replace (assume later added modes are }
          { better)                                                       }
          if assigned(newList) and
             (newList^.Mode^.maxx = mode.maxx) and
             (newList^.Mode^.maxy = mode.maxy) then
=======
=======
>>>>>>> origin/fixes_2_2
          list := newModeList.modeinfo[driverNr];
          { sort first by x resolution, then by yresolution }
          while assigned(list) and
                ((list^.maxx < mode.maxx) or
                 ((list^.maxx = mode.maxx) and
                  (list^.maxy < mode.maxy))) do
            begin
              prev := list;
              list := list^.next;
            end;
          { mode already exists? -> replace (assume later added modes are }
          { better)                                                       }
          if assigned(list) and
             (list^.maxx = mode.maxx) and
             (list^.maxy = mode.maxy) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            begin
{$ifdef logging}
              logln('replacing resolution '+strf(modenr)+' for drivernr '+strf(drivernr)+
                ' ('+strf(mode.maxx)+'x'+strf(mode.maxy)+')');
{$endif logging}
              { save/restore next, drivernr and drivermode in list }
<<<<<<< HEAD
<<<<<<< HEAD
              prev := newList^.next;
              newList^.Mode := newMode;
              newList^.next := prev;
=======
              prev := list^.next;
              list^ := mode;
              list^.next := prev;
>>>>>>> graemeg/fixes_2_2
=======
              prev := list^.next;
              list^ := mode;
              list^.next := prev;
>>>>>>> origin/fixes_2_2
            end
          else
            begin
              new(newLst);
              { Increase the number of modes for this driver }
<<<<<<< HEAD
<<<<<<< HEAD
              newLst^.Mode := newMode;
=======
              newLst^ := mode;
>>>>>>> graemeg/fixes_2_2
=======
              newLst^ := mode;
>>>>>>> origin/fixes_2_2
{$ifdef logging}
              logln('Adding resolution '+strf(modenr)+' for drivernr '+strf(drivernr)+
                ' ('+strf(mode.maxx)+'x'+strf(mode.maxy)+')');
{$endif logging}
<<<<<<< HEAD
<<<<<<< HEAD
              newLst^.next := newList;
=======
              newLst^.next := list;
>>>>>>> graemeg/fixes_2_2
=======
              newLst^.next := list;
>>>>>>> origin/fixes_2_2
              if assigned(prev) then
                prev^.next := newLst
              else
                newModeList.modeinfo[driverNr] := newLst;
            end;
        end;
       { renumber internmodenumber }
<<<<<<< HEAD
<<<<<<< HEAD
       newList := newModeList.modeinfo[driverNr];
       i:=0;
       while assigned(newList) do
        begin
          inc(i);
          newList^.internmodenumber:=i;
          newList:=newList^.next;
=======
=======
>>>>>>> origin/fixes_2_2
       list := newModeList.modeinfo[driverNr];
       i:=0;
       while assigned(list) do
        begin
          inc(i);
          list^.internmodenumber:=i;
          list:=list^.next;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        end;
       newModeList.loHiModeNr[driverNr].lo:=1;
       newModeList.loHiModeNr[driverNr].hi:=i;
     end;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
    { TP-like mode stuff }
    if not assigned(ModeList) then
      begin
        new(ModeList);
        System.move(mode, ModeList^, sizeof(Mode));
      end
    else
      begin
        list := ModeList;
        { go to the end of the list }
        while assigned(list^.next) do
          list:=list^.next;
        new(NewLst);
        list^.next := NewLst;
        System.move(mode, NewLst^, sizeof(Mode));
      end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;


  procedure initmode(var mode: TModeInfo);
  {********************************************************}
  { Procedure InitMode()                                   }
  {--------------------------------------------------------}
  { This routine initialized the mode to default values.   }
  {********************************************************}
   begin
     FillChar(mode,sizeof(Mode),#0);
   end;


   function searchmode(ReqDriver : smallint; var reqmode: smallint): PModeInfo;
  {********************************************************}
  { Procedure SearchMode()                                 }
  {--------------------------------------------------------}
  { This routine searches the list of recognized modes,    }
  { and tries to find the <reqmode> in the <reqdriver>     }
  { return nil if not found, otherwise returns the found   }
  { structure.                                             }
  { note: if reqmode = -32768, the first mode available    }
  {   for reqdriver is returned (JM)                       }
  {       if reqmode = -32767, the last mode available     }
  {   for reqdriver is returned (JM)                       }
  {********************************************************}
    var
     list, lastModeInfo: PModeInfo;
<<<<<<< HEAD
<<<<<<< HEAD
     newList: PNewModeList;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     x,y: longint;
    begin
{$ifdef logging}
      LogLn('Searching for driver '+strf(reqdriver)+' and mode '+strf(reqmode));
{$endif logging}
      if (reqDriver >= lowNewDriver) and
         (reqDriver <= highNewDriver) then
        begin
          case reqMode of
            -32768:
              begin
                reqMode := newModeList.loHiModeNr[reqDriver].lo;
<<<<<<< HEAD
<<<<<<< HEAD
                if newModeList.modeinfo[reqDriver] <> nil then
                  searchMode := newModeList.modeinfo[reqDriver]^.Mode
                else
                  searchMode := nil;
=======
                searchMode := newModeList.modeinfo[reqDriver];
>>>>>>> graemeg/fixes_2_2
=======
                searchMode := newModeList.modeinfo[reqDriver];
>>>>>>> origin/fixes_2_2
              end;
            -32767:
              begin
                reqMode := newModeList.loHiModeNr[reqDriver].hi;
                searchMode := nil;
                { Are there any modes available for this driver? }
                if reqMode <> -1 then
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
                    newList := newModeList.modeinfo[reqDriver];
                    while assigned(newList^.next) do
                      newList := newList^.next;
                    searchMode := newList^.Mode;
=======
=======
>>>>>>> origin/fixes_2_2
                    list := newModeList.modeinfo[reqDriver];
                    while assigned(list^.next) do
                      list := list^.next;
                    searchMode := list;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                  end;
              end;
            else
              begin
<<<<<<< HEAD
<<<<<<< HEAD
                newList := newModeList.modeinfo[reqDriver];
                searchMode := nil;
                if not assigned(newList) then
=======
                list := newModeList.modeinfo[reqDriver];
                searchMode := nil;
                if not assigned(list) then
>>>>>>> graemeg/fixes_2_2
=======
                list := newModeList.modeinfo[reqDriver];
                searchMode := nil;
                if not assigned(list) then
>>>>>>> origin/fixes_2_2
                  exit;
                if mode2res(reqMode,x,y) then
                  begin
                    x := pred(x);
                    y := pred(y);
<<<<<<< HEAD
<<<<<<< HEAD
                    while assigned(newList) and
                          ((newList^.Mode^.maxx < x) or
                           ((newList^.Mode^.maxx = x) and
                            (newList^.Mode^.maxy < y))) do
                      newList := newList^.next;
                    if not assigned(newList) or
                       (newList^.Mode^.maxx <> x) or
                       (newList^.Mode^.maxy <> y) then
                      newList := nil;
                    if newList <> nil then
                      searchmode := newList^.Mode
                    else
                      searchmode := nil;
                  end
                else
                  begin
                    while assigned(newList) and
                          (newList^.internModeNumber <> reqMode) do
                      newList := newList^.next;
                    if newList <> nil then
                      searchMode := newList^.Mode
                    else
                      searchMode := nil;
=======
=======
>>>>>>> origin/fixes_2_2
                    while assigned(list) and
                          ((list^.maxx < x) or
                           ((list^.maxx = x) and
                            (list^.maxy < y))) do
                      list := list^.next;
                    if not assigned(list) or
                       (list^.maxx <> x) or
                       (list^.maxy <> y) then
                      list := nil;
                    searchmode := list;
                  end
                else
                  begin
                    while assigned(list) and
                          (list^.internModeNumber <> reqMode) do
                      list := list^.next;
                    searchMode := list;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                  end;

              end;
          end;
          exit;
        end;
       searchmode := nil;
       list := ModeList;
       If assigned(list) then
         lastModeInfo := list;
       { go to the end of the list }
       while assigned(list) do
         begin
{$ifdef logging}
           Log('Found driver '+strf(list^.DriverNumber)+
               ' and mode $'+hexstr(list^.ModeNumber,4)+'...');
{$endif logging}
             if ((list^.DriverNumber = ReqDriver) and
                 ((list^.ModeNumber = ReqMode) or
                  { search for lowest mode }
                  (reqMode = -32768))) or
                 { search for highest mode }
                ((reqMode = -32767) and
                 (lastModeInfo^.driverNumber = reqDriver) and
                 ((list^.driverNumber <> lastModeInfo^.driverNumber) or
                   not(assigned(list^.next)))) then
               begin
{$ifdef logging}
                 LogLn('Accepted!');
{$endif logging}
                 searchmode := list;
                 If reqMode = -32768 then
                   reqMode := list^.ModeNumber
                 else if reqMode = -32767 then
                   begin
                     reqMode := lastModeInfo^.ModeNumber;
                     searchMode := lastModeInfo;
                   end;
                 exit;
               end;
{$ifdef logging}
             LogLn('Rejected.');
{$endif logging}
             lastModeInfo := list;
             list:=list^.next;
          end;
    end;


{-----------------------------------------------------------------------}
{                          External routines                            }
{-----------------------------------------------------------------------}

   function GetModeName(ModeNumber: smallint): string;
  {********************************************************}
  { Function GetModeName()                                 }
  {--------------------------------------------------------}
  { Checks  the known video list, and returns ModeName     }
  { string. On error returns an empty string.              }
  {********************************************************}
    var
     mode: PModeInfo;
    begin
      mode:=nil;
      GetModeName:='';
      { only search in the current driver modes ... }
<<<<<<< HEAD
<<<<<<< HEAD
      if (ModeNumber >= lowNewMode) and (ModeNumber <= highNewMode) then
          mode:=SearchMode(IntCurrentNewDriver,ModeNumber)
      else
          mode:=SearchMode(IntCurrentDriver,ModeNumber);
=======
      mode:=SearchMode(IntCurrentNewDriver,ModeNumber);
>>>>>>> graemeg/fixes_2_2
=======
      mode:=SearchMode(IntCurrentNewDriver,ModeNumber);
>>>>>>> origin/fixes_2_2
      if assigned(mode) then
          GetModeName:=Mode^.ModeName
      else
         _GraphResult := grInvalidMode;
    end;

   function GetGraphMode: smallint;
     begin
      GetGraphMode := IntCurrentMode;
     end;

   function GetMaxMode: smallint;
   { I know , i know, this routine is very slow, and it would }
   { be much easier to sort the linked list of possible modes }
   { instead of doing this, but I'm lazy!! And anyways, the   }
   { speed of the routine here is not that important....      }
    var
     i: smallint;
     mode: PModeInfo;
    begin
      mode:=nil;
      i:=0;
      repeat
        inc(i);
        { mode 0 always exists... }
        { start search at 1..     }
        mode:=SearchMode(IntCurrentNewDriver,i);
      until not assigned(mode);
      GetMaxMode:=i;
    end;


    procedure GetModeRange(GraphDriver: smallint; var LoMode,
      HiMode: smallint);
      var
       mode : PModeInfo;
     begin
       {$ifdef logging}
       LogLn('GetModeRange : Enter ('+strf(GraphDriver)+')');
       {$endif}
       HiMode:=-1;
       mode := nil;
       { First search if the graphics driver is supported ..  }
       { since mode zero is always supported.. if that driver }
       { is supported it should return something...           }

       { not true, e.g. VESA doesn't have a mode 0. Changed so}
       { -32768 means "return lowest mode in second parameter }
       { also, under VESA some modes may not be supported     }
       { (e.g. $108 here) while some with a higher number can }
       { be supported ($112 and onward), so I also added that }
       { -32767 means "return highest mode in second parameter}
       { This whole system should be overhauled though to work}
       { without such hacks (JM)                              }
       loMode := -32768;
       mode := SearchMode(GraphDriver, loMode);
       { driver not supported...}
       if not assigned(mode) then
         begin
           loMode := -1;
           exit;
         end;
       {$ifdef logging}
       LogLn('GetModeRange : Mode '+strf(lomode)+' found');
       {$endif}
       { now it exists... find highest available mode... }
       hiMode := -32767;
       mode:=SearchMode(GraphDriver,hiMode);
     end;


  procedure SetGraphMode(mode: smallint);
    var
     modeinfo: PModeInfo;
     usedDriver: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
     dummyNewModeNr: smallint;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    begin
      { check if the mode exists... }
      { Depending on the modenumber, we search using the old or new }
      { graphdriver number (because once we entered graphmode,      }
      { getgraphmode() returns the old mode number and              }
      { both setgraphmode(getgraphmode) and setgraphmode(mAAAxBBB)  }
      { have to work (JM)                                           }
      case mode of
        detectMode:
          begin
            mode := -32767;
            usedDriver := IntcurrentNewDriver;
            modeInfo := searchmode(IntcurrentNewDriver,mode);
          end;
        lowNewMode..highNewMode:
          begin
            usedDriver := IntcurrentNewDriver;
            modeInfo := searchmode(IntcurrentNewDriver,mode);
          end;
        else
          begin
            usedDriver := IntcurrentDriver;
            modeinfo := searchmode(IntcurrentDriver,mode);
          end;
      end;
      if not assigned(modeinfo) then
        begin
{$ifdef logging}
          LogLn('Mode setting failed in setgraphmode pos 1');
{$endif logging}
          _GraphResult := grInvalidMode;
          exit;
       end;
    { reset all hooks...}
    DefaultHooks;

      { required hooks - returns error if no hooks to these }
      { routines.                                           }
      if assigned(modeinfo^.DirectPutPixel) then
         DirectPutPixel := modeinfo^.DirectPutPixel
      else
        begin
{$ifdef logging}
          LogLn('Mode setting failed in setgraphmode pos 2');
{$endif logging}
         DefaultHooks;
         _Graphresult := grInvalidMode;
         exit;
       end;

      if assigned(modeinfo^.PutPixel) then
         PutPixel := modeinfo^.PutPixel
      else
        begin
{$ifdef logging}
          LogLn('Mode setting failed in setgraphmode pos 3');
{$endif logging}
         DefaultHooks;
         _Graphresult := grInvalidMode;
         exit;
       end;

      if assigned(modeinfo^.GetPixel) then
         GetPixel := modeinfo^.GetPixel
      else
        begin
{$ifdef logging}
          LogLn('Mode setting failed in setgraphmode pos 4');
{$endif logging}
         DefaultHooks;
         _Graphresult := grInvalidMode;
         exit;
       end;

     if assigned(modeinfo^.SetRGBPalette) then
         SetRGBPalette := modeinfo^.SetRGBPalette
     else
        begin
{$ifdef logging}
          LogLn('Mode setting failed in setgraphmode pos 5');
{$endif logging}
         DefaultHooks;
         _Graphresult := grInvalidMode;
         exit;
       end;

     if assigned(modeinfo^.GetRGBPalette) then
         GetRGBPalette := modeinfo^.GetRGBPalette
     else
        begin
{$ifdef logging}
          LogLn('Mode setting failed in setgraphmode pos 6');
{$endif logging}
         DefaultHooks;
         _Graphresult := grInvalidMode;
         exit;
       end;

      { optional hooks. }
      if assigned(modeinfo^.SetAllPalette) then
        SetAllPalette := modeinfo^.SetAllPalette;

      if assigned(modeinfo^.ClearViewPort) then
         ClearViewPort := modeinfo^.ClearViewPort;
      if assigned(modeinfo^.PutImage) then
         PutImage := modeinfo^.PutImage;
      if assigned(modeinfo^.GetImage) then
         GetImage := modeinfo^.GetImage;
      if assigned(modeinfo^.ImageSize) then
         ImageSize := modeinfo^.ImageSize;
      if assigned(modeinfo^.GetScanLine) then
         GetScanLine := modeinfo^.GetScanLine;
      if assigned(modeinfo^.Line) then
         Line := modeinfo^.Line;
      if assigned(modeinfo^.InternalEllipse) then
         InternalEllipse := modeinfo^.InternalEllipse;
      if assigned(modeinfo^.PatternLine) then
         PatternLine := modeinfo^.PatternLine;
      if assigned(modeinfo^.HLine) then
         Hline := modeinfo^.Hline;
      if assigned(modeinfo^.Vline) then
         VLine := modeinfo^.VLine;
      if assigned(modeInfo^.SetVisualPage) then
         SetVisualPage := modeInfo^.SetVisualPage;
      if assigned(modeInfo^.SetActivePage) then
         SetActivePage := modeInfo^.SetActivePage;
      if assigned(modeInfo^.OutTextXY) then
         OutTextXY:=modeInfo^.OutTextXY;
<<<<<<< HEAD
<<<<<<< HEAD
      if assigned(modeInfo^.SetBkColor) then
         SetBkColor:=modeInfo^.SetBkColor;
      if assigned(modeInfo^.GetBkColor) then
         GetBkColor:=modeInfo^.GetBkColor;

      IntCurrentMode := modeinfo^.ModeNumber;
      { fix by nickysn: always set *both* IntCurrentDriver and IntCurrentNewDriver to the
        *correct* old & new style driver numbers. Solves a lot of bugs, e.g. when using InitGraph
	with a new style driver/mode and then later using SetGraphMode(GetGraphMode) }
      IntCurrentDriver := modeinfo^.DriverNumber;
      res2Mode(modeinfo^.maxx+1, modeinfo^.maxy+1, modeinfo^.maxColor, IntCurrentNewDriver, dummyNewModeNr);
=======

      IntCurrentMode := modeinfo^.ModeNumber;
      IntCurrentDriver := usedDriver;
>>>>>>> graemeg/fixes_2_2
=======

      IntCurrentMode := modeinfo^.ModeNumber;
      IntCurrentDriver := usedDriver;
>>>>>>> origin/fixes_2_2
{$ifdef logging}
       logln('Entering mode '+strf(intCurrentMode)+' of driver '+strf(intCurrentDriver));
{$endif logging}
      XAspect := modeinfo^.XAspect;
      YAspect := modeinfo^.YAspect;
      MaxX := modeinfo^.MaxX;
      MaxY := modeinfo^.MaxY;
{$ifdef logging}
      logln('maxx = '+strf(maxx)+', maxy = '+strf(maxy));
{$endif logging}
      HardwarePages := modeInfo^.HardwarePages;
      MaxColor := modeinfo^.MaxColor;
      PaletteSize := modeinfo^.PaletteSize;
      { is this a direct color mode? }
      DirectColor := modeinfo^.DirectColor;
      { now actually initialize the video mode...}
      { check first if the routine exists        }
      if not assigned(modeinfo^.InitMode) then
        begin
{$ifdef logging}
          LogLn('Mode setting failed in setgraphmode pos 7');
{$endif logging}
          DefaultHooks;
          _GraphResult := grInvalidMode;
          exit;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
      _GraphResult := grOk;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      modeinfo^.InitMode;
      if _GraphResult <> grOk then
        begin
          DefaultHooks;
          exit;
        end;
      isgraphmode := true;
      { It is very important that this call be made }
      { AFTER the other variables have been setup.  }
      { Since it calls some routines which rely on  }
      { those variables.                            }
      SetActivePage(0);
      SetVisualPage(0);
      SetViewPort(0,0,MaxX,MaxY,TRUE);
      GraphDefaults;
    end;

    procedure RestoreCrtMode;
  {********************************************************}
  { Procedure RestoreCRTMode()                             }
  {--------------------------------------------------------}
  { Returns to the video mode which was set before the     }
  { InitGraph. Hardware state is set to the old values.    }
  {--------------------------------------------------------}
  { NOTE: -                                                }
  {       -                                                }
  {********************************************************}
     begin
       isgraphmode := false;
       RestoreVideoState;
     end;

