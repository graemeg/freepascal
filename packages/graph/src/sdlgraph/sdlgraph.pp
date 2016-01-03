{**************************************************************************
    Copyright            : (c) 2007 Evgeniy Ivanov
    email                : lolkaantimat@gmail.com
    http                 : Not yet
****************************************************************************/

    This file implements the sdl support for the graph unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
****************************************************************************}



{Important TODO list

TODO: color conversion. from pascal color constant to color (pixel) from SDL_putpixel. At this moment you may gent another colors than you're waiting for. The worsest thing is it may be black on the black

TODO: check all mode.HardwarePages and find true value for each mode!

TODO: check initgraph(0,0,' ') and work with modes to set up best SDL mode. 
Maybe to hook internDetectGraph and detectGraph. I thing they are not needed for sdlgraph

TODO: check VESA modes ModeNumber (if they needed)
}




{ Programming Notes
TODO: (it's a linux note, I haven't yet tested it on other systems) When SDL window is activated from the terminal readln does'n work!!! And when from KDE it autocloses without waiting readln input. I think that there must be a hook for readln while sdlgraph is active

TODO: setcaption for the programme. There is no such procedure in Graph unit so we should add it or use setcaption with default (prograamme's name)

TODO: to not forget about rgb_from_color's RGB values. Now they are just for testing!!!

TODO: configure refresh (flip) time. SDL_AddTimer(100...: time interval must be the same as monitor vertical refresh.

}


unit sdlgraph;

{$ifdef darwin}
{$linkframework Cocoa}
{$linklib SDLmain}
{$linklib gcc}
{$endif}

interface
//procedure sdlgraph_bar3D(x1, y1, x2, y2 : smallint;depth : word;top : boolean);
{$i graphh.inc}

const

{==================================================================================================================================================
Graphics Drivers Constants. Needed to support turbo pascale code  TODO: is it needed???
It's highly recommended to use Detect (0 constant) for grDriver and grmode: initGraph(0,0,' ') to allow SDL to configure app for the best perfomance
====================================================================================================================================================
}

 //Detect      =0;     is in the graphh.inc
<<<<<<< HEAD
<<<<<<< HEAD
 //CGA           =1;   is in graphh.inc
 //MCGA	       =2;   is in graphh.inc
 //EGA           =3;   is in graphh.inc
 //EGA64         =4;   is in graphh.inc
 //EGAMono       =5;   is in graphh.inc
=======
=======
>>>>>>> origin/fixes_2_2
 CGA           =1;
 MCGA	       =2;
 EGA           =3;
 EGA64         =4;
 EGAMono       =5;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 IBM8514       =6;
 //HercMono      =7;   is in the graphh.inc
 ATT400        =8;
 //VGA           =9;   is in graphh.inc
 PC3270        =10;

{Graphics Modes for Each Driver}
<<<<<<< HEAD
<<<<<<< HEAD
 //CGAC0         =0;   is in graphh.inc
 //CGAC          =1;   is in graphh.inc
 //CGAC2         =2;   is in graphh.inc
 //CGAC3         =3;   is in graphh.inc
 //CGAHi         =4;   is in graphh.inc
 
 //MCGAC0        =0;   is in graphh.inc
 //MCGAC         =1;   is in graphh.inc
 //MCGAC2        =2;   is in graphh.inc
 //MCGAC3        =3;   is in graphh.inc
 //MCGAMed       =4;   is in graphh.inc
 //MCGAHi        =5;   is in graphh.inc
 
 //EGAMonoHi     =3;   is in graphh.inc
=======
=======
>>>>>>> origin/fixes_2_2
 CGAC0         =0;
 CGAC          =1;
 CGAC2         =2;
 CGAC3         =3;
 CGAHi         =4;
 
 MCGAC0        =0;
 MCGAC         =1;
 MCGAC2        =2;
 MCGAC3        =3;
 MCGAMed       =4;
 MCGAHi        =5;
 
 EGAMonoHi     =3;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 //HercMonoHi    =0;      is in the graphh.inc
 //VGALo         =0;      is in the graphh.inc
 //VGAMed        =1;      is in the graphh.inc
 //VGAHi         =2;      is in the graphh.inc
 
<<<<<<< HEAD
<<<<<<< HEAD
 //EGALo         =0;   is in graphh.inc
 //EGAHi         =1;   is in graphh.inc
 //EGA64Lo       =0;   is in graphh.inc
 //EGA64Hi       =1;   is in graphh.inc
=======
=======
>>>>>>> origin/fixes_2_2
 EGALo         =0;
 EGAHi         =1;
 EGA64Lo       =0;
 EGA64Hi       =1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 
 ATT400C0      =0;
 ATT400C1      =1;
 ATT400C2      =2;
 ATT400C3      =3;
 ATT400CMed    =4;
 ATT400Hi      =5;
 
 IBM8514Lo     =0;
 IBM8514Hi     =1;
 
 PC3270Hi      =0;

{ From *Go32* VESA Specific video modes. }
  m320x200x32k      = $10D;
  m320x200x64k      = $10E;

  m640x400x256      = $100;

  m640x480x256      = $101;
  m640x480x32k      = $110;
  m640x480x64k      = $111;

  m800x600x16       = $102;
  m800x600x256      = $103;
  m800x600x32k      = $113;
  m800x600x64k      = $114;

  m1024x768x16      = $104;
  m1024x768x256     = $105;
  m1024x768x32k     = $116;
  m1024x768x64k     = $117;

  m1280x1024x16     = $106;
  m1280x1024x256    = $107;
  m1280x1024x32k    = $119;
  m1280x1024x64k    = $11A;
implementation

uses {$ifdef unix}cthreads,{$endif}
   sdl,sdlutils,
          logger,
          SysUtils;

const
  InternalDriverName = 'SDL';

{$define sdlgraph}

{$i graph.inc}

var
screen: PSDL_Surface; //Global becouse its value is needed by some functions



procedure CloseGraph;
begin
 If not isgraphmode then
 begin
  _graphresult := grnoinitgraph;
 exit;
 end;
   isgraphmode := false;
   SDL_Quit();
 //Halt(0);   TODO: check, if it close application wich calls sdlgraph
end;


procedure Slock;
begin
  if SDL_MUSTLOCK(screen) then
    if SDL_LockSurface(screen) < 0 then
          begin
          Log.LogError( Format( 'Cant lock screen: : %s', [SDL_GetError]), 'Slock' );
          CloseGraph;
          end;
end;


function timer_flip(flip_interval:Uint32; flip_callback_param:Pointer):Uint32;
begin
SDL_Flip(screen);
timer_flip:=flip_interval;
exit;
end;


{
procedure Slock;
begin
  if SDL_MUSTLOCK(screen) then
    if SDL_LockSurface(screen) < 0 then
          begin
          Log.LogError( Format( 'Cant lock screen: : %s', [SDL_GetError]), 'Slock' );
          CloseGraph;
          end;
end;


procedure Sulock; //Unlock and flip the surface
begin
  if SDL_MUSTLOCK(screen) then 
    SDL_UnlockSurface(screen);
  SDL_Flip(screen);

end;

}



{PutPixel and GetPixel use SDL_* functions from sdlutils unit.}

procedure sdlgraph_PutPixel(X,Y:smallint; color: Word);
begin
 X:= X + StartXViewPort;
 Y:= Y + StartYViewPort;

{ convert to absolute coordinates and then verify clipping...}
 if ClipPixels then
   begin
   if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
          exit;
   if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
          exit;
   end;
 SDL_PutPixel(screen,x,y,255);
 exit;

end;



function sdlgraph_GetPixel(X,Y:smallint):Word;
var
temp:word;
begin
 X:= X + StartXViewPort;
 Y:= Y + StartYViewPort;
 temp:=word(SDL_GetPixel(screen,x,y));
 sdlgraph_GetPixel:=temp;
 exit;
end;


procedure sdlgraph_DirectPutPixel(X,Y: smallint);
var
Color: word;
begin

    case CurrentWriteMode of
      XORPut:
        begin
{ getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor Xor Color;
        end;
      OrPut:
        begin
{ getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor Or Color;
        end;
      AndPut:
        begin
{ getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor And Color;
        end;
      NotPut:
        begin
          Color := (Not CurrentColor) and 15;
        end
      else
        Color := CurrentColor;
   end;

sdlgraph_PutPixel(X,Y,Color);

exit;
end;


{
procedure nonBuf_DirectPutPixel(X,Y: smallint);   // for HLine and other such hooks. It doesn't use buffering (SLock & Sulock routines).
begin
  CurrentColor:=255;
  SDL_PutPixel(screen,x,y,CurrentColor);
exit;
end;


procedure sdlgraph_HLine(x,x2,y: smallint);
var
temp:DefPixelProc;
begin
 HLineDefault(x,x2,y);
 //SDL_DrawLine(screen,X,y,x2,y,255);
end;

procedure sdlgraph_VLine(x,y,y2:smallint);
var
temp:DefPixelProc;
begin
VLineDefault(x,y,y2);
end;



procedure sdlgraph_line(X1, Y1, X2, Y2: smallint);
var
temp:DefPixelProc;
begin
 LineDefault(X1, Y1, X2, Y2);
 //SDL_DrawLine(screen,X1,y1,x2,y2,255);
end;
}

{
procedure nonBuf_HLine(x,x2,y: smallint);
var
temp:DefPixelProc;
begin
 temp:=DirectPutPixel;
 DirectPutPixel:=@nonBuf_DirectPutPixel;
 HLineDefault(x,x2,y);
 DirectPutPixel:=temp;
end;

procedure nonBuf_VLine(x,y,y2:smallint);
var
temp:DefPixelProc;
begin
 temp:=DirectPutPixel;
 DirectPutPixel:=@nonBuf_DirectPutPixel;
 VLineDefault(x,y,y2);
 DirectPutPixel:=temp;
end;
}


{
procedure nonBuf_line(X1, Y1, X2, Y2: smallint);
var
temp:DefPixelProc;
begin
 temp:=DirectPutPixel;
 DirectPutPixel:=@nonBuf_DirectPutPixel;
 LineDefault(X1, Y1, X2, Y2);
 DirectPutPixel:=temp;
end;
}

{
procedure sdlgraph_InternalEllipse(X,Y: smallint;XRadius: word;
    YRadius:word; stAngle,EndAngle: word; pl: PatternLineProc);
var
temp:PutPixelProc;
begin
 InternalEllipseDefault(X,Y,XRadius,YRadius,stAngle,EndAngle,pl);
end;
}


procedure InitSDLgraph(Width,Height,BPP:Integer);
var
 videoflags : Uint32;
 videoInfo : PSDL_VideoInfo;
 
 flip_callback_param:Pointer;
 flip_timer_id:PSDL_TimerID;
begin
  if ( SDL_Init( SDL_INIT_TIMER or SDL_INIT_VIDEO ) < 0 ) then
  begin
    Log.LogError( Format( 'Could not initialize SDL : %s', [SDL_GetError] ), 'InitSDLgraph' );
    exit;
  end;


  // Fetch the video info 
  videoInfo := SDL_GetVideoInfo;

  if ( videoInfo = nil ) then
  begin
    Log.LogError( Format( 'Video query failed : %s', [SDL_GetError] ), 'InitSDLgraph' );
    CloseGraph;
    exit;
  end;


  // the flags to pass to SDL_SetVideoMode 
  videoFlags := SDL_DOUBLEBUF; // Enable double buffering 
  videoFlags := videoFlags or SDL_HWPALETTE; // Store the palette in hardware 

  // This checks to see if surfaces can be stored in memory 
 if  videoInfo^.hw_available <> 0  then
    videoFlags := videoFlags or SDL_HWSURFACE
  else
    videoFlags := videoFlags or SDL_SWSURFACE;

  // This checks if hardware blits can be done * /
  if videoInfo^.blit_hw <> 0 then
    videoFlags := videoFlags or SDL_HWACCEL;
  videoflags := videoFlags or SDL_RESIZABLE;    // Enable window resizing    TODO: Do we want to have it in graph module?


  if (SDL_VideoModeOK(Width,Height,BPP,videoFlags) = 0) then
     begin
     //TODO: create 1 string from parametres!
     //Log.LogError('InitSDLgraph: ',Width,'x',Height,'x',BPP,' - no such mode (also you may check videoflags in the sdlgraph unit (procedure InitSDLgraph)');
     exit;
     end;

  screen := SDL_SetVideoMode(Width, Height, BPP, SDL_SWSURFACE );   // TODO: use videoflags but not SDL_SWSURFACE!
    
//It doesn't work yet!
{if ( surface = nil ) then
  begin
    Log.LogError( Format( 'Unable to SetVideMode : %s', [SDL_GetError]), 'InitSDLgraph' );
    InitSDLgraph:=false;
    exit;
    CloseGraph;
  end;}


flip_timer_id := SDL_AddTimer(100,TSDL_NewTimerCallback( @timer_flip ), nil ); //TODO: time interval must be the same as monitor vertical refresh
end;

procedure sdlgraph_Init1280x1024x64k;
begin
 InitSDLgraph(1280,1024,16);
end;


procedure sdlgraph_Init1280x1024x32k;
begin
 InitSDLgraph(1280,1024,15);   //TODO: maybe to set 16 bit??? It's about all 15bpp modes.
end;

procedure sdlgraph_Init1280x1024x256;
begin
 InitSDLgraph(1280,1024,8);
end;

procedure sdlgraph_Init1280x1024x16;
begin
 InitSDLgraph(1280,1024,4);
end;

procedure sdlgraph_Init1024x768x64k;
begin
 InitSDLgraph(1024,768,16);
end;

procedure sdlgraph_Init640x480x32k;
begin
 InitSDLgraph(640,480,15);
end;

procedure sdlgraph_Init1024x768x256;
begin
 InitSDLgraph(1024,768,8);
end;

procedure sdlgraph_Init1024x768x16;
begin
 InitSDLgraph(1024,768,4);
end;

procedure sdlgraph_Init800x600x64k;
begin
 InitSDLgraph(800,600,16);
end;

procedure sdlgraph_Init800x600x32k;
begin
 InitSDLgraph(800,600,15);
end;

procedure sdlgraph_Init800x600x256;
begin
 InitSDLgraph(800,600,8);
end;

procedure sdlgraph_Init800x600x16;
begin
 InitSDLgraph(800,600,4);
end;

procedure sdlgraph_Init640x480x64k;
begin
 InitSDLgraph(640,480,16);
end;

procedure sdlgraph_Init1024x768x32k;
begin
 InitSDLgraph(1024,768,15);
end;

procedure sdlgraph_Init640x480x256;
begin
 InitSDLgraph(640,480,8);
end;

procedure sdlgraph_Init640x400x256;
begin
 InitSDLgraph(640,400,8);
end;

procedure sdlgraph_Init320x200x64k;
begin
 InitSDLgraph(320,200,16);
end;

procedure sdlgraph_Init320x200x32k;
begin
 InitSDLgraph(320,200,15);
end;

procedure sdlgraph_Init640x480x16;
begin
 InitSDLgraph(640,480,4);
end;

procedure sdlgraph_Init640x350x16;
begin
 InitSDLgraph(640,350,4);
end;

procedure sdlgraph_Init640x200x16;
begin
 InitSDLgraph(640,200,4);
end;

procedure sdlgraph_InitModeX;
begin
 InitSDLgraph(320,200,8);
end;

procedure sdlgraph_Init320;
begin
 InitSDLgraph(320,200,8);
end;




//TODO Check what does it do and if it is needed
//BEGIN TODO
procedure savestate;

  begin
  end;

procedure restorestate;

  begin
  end;

procedure sdlgraph_SetRGBpalette(ColorNum, RedValue, GreenValue, BlueValue: smallint);
   begin
   end;

procedure sdlgraph_GetRGBpalette(ColorNum: smallint; var RedValue, GreenValue, BlueValue: smallint);
   begin
   end;


//END TODO


function QueryAdapterInfo:PModeInfo;
var
mode: TModeInfo;

   procedure setupSDLgraphDefaults;
    begin
    mode.DirectPutPixel:={$ifdef fpc}@{$endif}sdlgraph_DirectPutPixel;
    mode.PutPixel:={$ifdef fpc}@{$endif}sdlgraph_PutPixel;
    mode.GetPixel:={$ifdef fpc}@{$endif}sdlgraph_GetPixel;
    mode.SetRGBPalette := {$ifdef fpc}@{$endif}sdlgraph_SetRGBpalette;
    mode.GetRGBPalette := {$ifdef fpc}@{$endif}sdlgraph_GetRGBpalette;
    //mode.InternalEllipse := {$ifdef fpc}@{$endif}sdlgraph_InternalEllipse;
    if ((mode.MaxX+1)*35=(mode.MaxY+1)*64) then
      mode.XAspect:=7750
    else if ((mode.MaxX+1)*20=(mode.MaxY+1)*64) then
      mode.XAspect:=4500
    else if ((mode.MaxX+1)*40=(mode.MaxY+1)*64) then
      mode.XAspect:=8333
    else { assume 4:3 }
      mode.XAspect:=10000;
    //mode.HLine:={$ifdef fpc}@{$endif}sdlgraph_HLine;
    //mode.VLine:={$ifdef fpc}@{$endif}sdlgraph_VLine;
    //mode.Line:={$ifdef fpc}@{$endif}sdlgraph_line;
    end;

begin
     QueryAdapterInfo := ModeList;
{ If the mode listing already exists... }
{ simply return it, without changing    }
{ anything...                           }
     if assigned(ModeList) then
       exit;

         SaveVideoState:={$ifdef fpc}@{$endif}savestate;
         RestoreVideoState:={$ifdef fpc}@{$endif}restorestate;







//    =======================================================MODES  FROM *GO32*=====================================================================
//    ==============================================================================================================================================

            InitMode(mode);
{ now add all standard VGA modes...       }
            mode.DriverNumber:= LowRes;
            mode.HardwarePages:= 0;
            mode.ModeNumber:=0;
            mode.ModeName:='320 x 200 VGA';
            mode.MaxColor := 256;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := FALSE;
            mode.MaxX := 319;
            mode.MaxY := 199;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init320;
            setupSDLgraphDefaults;
            AddMode(mode);

{ now add all standard VGA modes...       }
            InitMode(mode);
            mode.DriverNumber:= LowRes;
            mode.ModeNumber:=1;
            mode.HardwarePages := 3; { 0..3 }
            mode.ModeName:='320 x 200 ModeX';
            mode.MaxColor := 256;
            mode.DirectColor := FALSE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 319;
            mode.MaxY := 199;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_InitModeX;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=VGALo;
            mode.DriverNumber := VGA;
            mode.ModeName:='640 x 200 VGA';
            mode.MaxColor := 16;
            mode.HardwarePages := 2;
            mode.DirectColor := FALSE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 639;
            mode.MaxY := 199;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init640x200x16;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=VGAMed;
            mode.DriverNumber := VGA;
            mode.ModeName:='640 x 350 VGA';
            mode.HardwarePages := 1;
            mode.MaxColor := 16;
            mode.DirectColor := FALSE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 639;
            mode.MaxY := 349;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init640x350x16;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=VGAHi;
            mode.DriverNumber := VGA;
            mode.HardwarePages := 0;
            mode.ModeName:='640 x 480 VGA';
            mode.MaxColor := 16;
            mode.DirectColor := FALSE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 639;
            mode.MaxY := 479;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init640x480x16;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m320x200x32k;
            mode.DriverNumber := VESA;
            mode.ModeName:='320 x 200 VESA';
            mode.MaxColor := 32768;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := TRUE;
            mode.MaxX := 319;
            mode.MaxY := 199;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init320x200x32k;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m320x200x64k;
            mode.DriverNumber := VESA;
            mode.ModeName:='320 x 200 VESA';
            mode.MaxColor := 65536;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := TRUE;
            mode.MaxX := 319;
            mode.MaxY := 199;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init320x200x64k;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m640x400x256;
            mode.DriverNumber := VESA;
            mode.ModeName:='640 x 400 VESA';
            mode.MaxColor := 256;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := FALSE;
            mode.MaxX := 639;
            mode.MaxY := 399;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init640x400x256;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m640x480x256;
            mode.DriverNumber := VESA;
            mode.ModeName:='640 x 480 VESA';
            mode.MaxColor := 256;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 639;
            mode.MaxY := 479;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init640x480x256;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m640x480x32k;
            mode.DriverNumber := VESA;
            mode.ModeName:='640 x 480 VESA';
            mode.MaxColor := 32768;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := TRUE;
            mode.MaxX := 639;
            mode.MaxY := 479;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init640x480x32k;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m640x480x64k;
            mode.DriverNumber := VESA;
            mode.ModeName:='640 x 480 VESA';
            mode.MaxColor := 65536;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := TRUE;
            mode.MaxX := 639;
            mode.MaxY := 479;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init640x480x64k;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m800x600x16;
            mode.DriverNumber := VESA;
            mode.ModeName:='800 x 600 VESA';
            mode.MaxColor := 16;
            mode.HardwarePages := 2;
            mode.DirectColor := FALSE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 799;
            mode.MaxY := 599;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init800x600x16;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m800x600x256;
            mode.DriverNumber := VESA;
            mode.ModeName:='800 x 600 VESA';
            mode.MaxColor := 256;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := FALSE;
            mode.MaxX := 799;
            mode.MaxY := 599;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init800x600x256;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m800x600x32k;
            mode.DriverNumber := VESA;
            mode.ModeName:='800 x 600 VESA';
            mode.MaxColor := 32768;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := TRUE;
            mode.MaxX := 799;
            mode.MaxY := 599;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init800x600x32k;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m800x600x64k;
            mode.DriverNumber := VESA;
            mode.ModeName:='800 x 600 VESA';
            mode.MaxColor := 65536;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := TRUE;
            mode.MaxX := 799;
            mode.MaxY := 599;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init800x600x64k;
            setupSDLgraphDefaults;
            AddMode(mode);
   
            InitMode(mode);
            mode.ModeNumber:=m1024x768x16;
            mode.DriverNumber := VESA;
            mode.ModeName:='1024 x 768 VESA';
            mode.MaxColor := 16;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := FALSE;
            mode.MaxX := 1023;
            mode.MaxY := 767;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init1024x768x16;
            setupSDLgraphDefaults;
            AddMode(mode);
            
            InitMode(mode);
            mode.ModeNumber:=m1024x768x256;
            mode.DriverNumber := VESA;
            mode.ModeName:='1024 x 768 VESA';
            mode.MaxColor := 256;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := FALSE;
            mode.MaxX := 1023;
            mode.MaxY := 767;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init1024x768x256;
            setupSDLgraphDefaults;
            AddMode(mode);
       
            InitMode(mode);
            mode.ModeNumber:=m1024x768x32k;
            mode.DriverNumber := VESA;
            mode.ModeName:='1024 x 768 VESA';
            mode.MaxColor := 32768;
            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.DirectColor := TRUE;
            mode.MaxX := 1023;
            mode.MaxY := 767;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init1024x768x32k;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m1024x768x64k;
            mode.DriverNumber := VESA;
            mode.ModeName:='1024 x 768 VESA';
            mode.MaxColor := 65536;
            mode.DirectColor := TRUE;

            mode.HardwarePages := 2;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 1023;
            mode.MaxY := 767;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init1024x768x64k;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m1280x1024x16;
            mode.DriverNumber := VESA;
            mode.ModeName:='1280 x 1024 VESA';
            mode.MaxColor := 16;
            mode.HardwarePages := 2;
            mode.DirectColor := FALSE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 1279;
            mode.MaxY := 1023;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init1280x1024x16;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m1280x1024x256;
            mode.DriverNumber := VESA;
            mode.ModeName:='1280 x 1024 VESA';
            mode.MaxColor := 256;
            mode.HardwarePages := 2;
            mode.DirectColor := FALSE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 1279;
            mode.MaxY := 1023;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init1280x1024x256;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m1280x1024x32k;
            mode.DriverNumber := VESA;
            mode.ModeName:='1280 x 1024 VESA';
            mode.MaxColor := 32768;
            mode.HardwarePages := 2;
            mode.DirectColor := TRUE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 1279;
            mode.MaxY := 1023;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init1280x1024x32k;
            setupSDLgraphDefaults;
            AddMode(mode);

            InitMode(mode);
            mode.ModeNumber:=m1280x1024x64k;
            mode.DriverNumber := VESA;
            mode.ModeName:='1280 x 1024 VESA';
            mode.MaxColor := 65536;
            mode.HardwarePages := 2;
            mode.DirectColor := TRUE;
            mode.PaletteSize := mode.MaxColor;
            mode.MaxX := 1279;
            mode.MaxY := 1023;
            mode.InitMode := {$ifdef fpc}@{$endif}sdlgraph_Init1280x1024x64k;
            setupSDLgraphDefaults;
            AddMode(mode);

end;






begin


InitializeGraph;

end.
