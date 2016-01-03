{$ifdef OGC_INTERFACE}


const
  VI_DISPLAY_PIX_SZ = 2;

  VI_INTERLACE     = 0;
  VI_NON_INTERLACE = 1;
  VI_PROGRESSIVE   = 2;

  VI_NTSC      = 0;
  VI_PAL       = 1;
  VI_MPAL      = 2;
  VI_DEBUG     = 3;
  VI_DEBUG_PAL = 4;
  VI_EURGB60   = 5;


  VI_XFBMODE_SF = 0;
  VI_XFBMODE_DF = 1;

  VI_FIELD_ABOVE = 1;
  VI_FIELD_BELOW = 0;

  // Maximum screen space
  VI_MAX_WIDTH_NTSC  = 720;
  VI_MAX_HEIGHT_NTSC = 480;

  VI_MAX_WIDTH_PAL   = 720;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  VI_MAX_HEIGHT_PAL  = 576;
=======
  VI_MAX_HEIGHT_PAL  = 574;
>>>>>>> graemeg/cpstrnew
=======
  VI_MAX_HEIGHT_PAL  = 574;
>>>>>>> graemeg/cpstrnew
=======
  VI_MAX_HEIGHT_PAL  = 574;
>>>>>>> graemeg/cpstrnew
=======
  VI_MAX_HEIGHT_PAL  = 574;
>>>>>>> origin/cpstrnew

  VI_MAX_WIDTH_MPAL  = 720;
  VI_MAX_HEIGHT_MPAL = 480;

  VI_MAX_WIDTH_EURGB60  = VI_MAX_WIDTH_NTSC;
  VI_MAX_HEIGHT_EURGB60 = VI_MAX_HEIGHT_NTSC;

function VIDEO_PadFramebufferWidth(width: cint): pcuint16; inline;


function VI_TVMODE(fmt, mode: cint): cint; inline;

{$define VI_TVMODE_NTSC_INT     := VI_TVMODE(VI_NTSC, VI_INTERLACE)}
{$define VI_TVMODE_NTSC_DS      := VI_TVMODE(VI_NTSC, VI_NON_INTERLACE)}
{$define VI_TVMODE_NTSC_PROG    := VI_TVMODE(VI_NTSC, VI_PROGRESSIVE)}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$define VI_TVMODE_PAL_INT := VI_TVMODE(VI_PAL, VI_INTERLACE)}
{$define VI_TVMODE_PAL_DS  := VI_TVMODE(VI_PAL, VI_NON_INTERLACE)}
{$define VI_TVMODE_PAL_PROG := VI_TVMODE(VI_PAL, VI_PROGRESSIVE)}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$define VI_TVMODE_NTSC_PROG_DS := VI_TVMODE(VI_NTSC, (VI_PROGRESSIVE or VI_NON_INTERLACE))}

{$define VI_TVMODE_PAL_INT := VI_TVMODE(VI_PAL, VI_INTERLACE)}
{$define VI_TVMODE_PAL_DS  := VI_TVMODE(VI_PAL, VI_NON_INTERLACE)}
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

{$define VI_TVMODE_EURGB60_INT     := VI_TVMODE(VI_EURGB60, VI_INTERLACE)}
{$define VI_TVMODE_EURGB60_DS      := VI_TVMODE(VI_EURGB60, VI_NON_INTERLACE)}
{$define VI_TVMODE_EURGB60_PROG    := VI_TVMODE(VI_EURGB60, VI_PROGRESSIVE)}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$define VI_TVMODE_MPAL_INT := VI_TVMODE(VI_MPAL, VI_INTERLACE)}
{$define VI_TVMODE_MPAL_DS  := VI_TVMODE(VI_MPAL, VI_NON_INTERLACE)}
{$define VI_TVMODE_MPAL_PROG := VI_TVMODE(VI_MPAL, VI_PROGRESSIVE)}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$define VI_TVMODE_EURGB60_PROG_DS := VI_TVMODE(VI_EURGB60, (VI_PROGRESSIVE or VI_NON_INTERLACE))}

{$define VI_TVMODE_MPAL_INT := VI_TVMODE(VI_MPAL, VI_INTERLACE)}
{$define VI_TVMODE_MPAL_DS  := VI_TVMODE(VI_MPAL, VI_NON_INTERLACE)}
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

{$define VI_TVMODE_DEBUG_INT := VI_TVMODE(VI_DEBUG, VI_INTERLACE)}

{$define VI_TVMODE_DEBUG_PAL_INT := VI_TVMODE(VI_DEBUG_PAL, VI_INTERLACE)}
{$define VI_TVMODE_DEBUG_PAL_DS  := VI_TVMODE(VI_DEBUG_PAL, VI_NON_INTERLACE)}


var
  TVNtsc240Ds: GXRModeObj; cvar; external;
  TVNtsc240DsAa: GXRModeObj; cvar; external;		
  TVNtsc240Int: GXRModeObj; cvar; external;			
  TVNtsc240IntAa: GXRModeObj; cvar; external;	
  TVNtsc480Int: GXRModeObj; cvar; external;			
  TVNtsc480IntDf: GXRModeObj; cvar; external;	
  TVNtsc480IntAa: GXRModeObj; cvar; external;	
  TVNtsc480Prog: GXRModeObj; cvar; external;  
  TVNtsc480ProgSoft: GXRModeObj; cvar; external;
  TVNtsc480ProgAa: GXRModeObj; cvar; external;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TVMpal240Ds: GXRModeObj; cvar; external;
  TVMpal240DsAa: GXRModeObj; cvar; external;
  TVMpal480IntDf: GXRModeObj; cvar; external;	
  TVMpal480IntAa: GXRModeObj; cvar; external;
  TVMpal480Prog: GXRModeObj; cvar; external;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TVMpal480IntDf: GXRModeObj; cvar; external;	
  TVMpal480IntAa: GXRModeObj; cvar; external;
  TVMpal240Ds: GXRModeObj; cvar; external;
  TVMpal240DsAa: GXRModeObj; cvar; external;
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
  TVPal264Ds: GXRModeObj; cvar; external;				
  TVPal264DsAa: GXRModeObj; cvar; external;			
  TVPal264Int: GXRModeObj; cvar; external;				
  TVPal264IntAa: GXRModeObj; cvar; external;		
  TVPal524IntAa: GXRModeObj; cvar; external;		
  TVPal528Int: GXRModeObj; cvar; external;				
  TVPal528IntDf: GXRModeObj; cvar; external;		
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TVPal576IntDfScale: GXRModeObj; cvar; external;
  TVPal576ProgScale: GXRModeObj; cvar; external;
=======
  TVPal574IntDfScale: GXRModeObj; cvar; external;
>>>>>>> graemeg/cpstrnew
=======
  TVPal574IntDfScale: GXRModeObj; cvar; external;
>>>>>>> graemeg/cpstrnew
=======
  TVPal574IntDfScale: GXRModeObj; cvar; external;
>>>>>>> graemeg/cpstrnew
=======
  TVPal574IntDfScale: GXRModeObj; cvar; external;
>>>>>>> origin/cpstrnew
  TVEurgb60Hz240Ds: GXRModeObj; cvar; external;
  TVEurgb60Hz240DsAa: GXRModeObj; cvar; external;
  TVEurgb60Hz240Int: GXRModeObj; cvar; external;
  TVEurgb60Hz240IntAa: GXRModeObj; cvar; external;
  TVEurgb60Hz480Int: GXRModeObj; cvar; external;
  TVEurgb60Hz480IntDf: GXRModeObj; cvar; external;
  TVEurgb60Hz480IntAa: GXRModeObj; cvar; external;
  TVEurgb60Hz480Prog: GXRModeObj; cvar; external;
  TVEurgb60Hz480ProgSoft: GXRModeObj; cvar; external;
  TVEurgb60Hz480ProgAa: GXRModeObj; cvar; external;
{$endif OGC_INTERFACE}


{$ifdef OGC_IMPLEMENTATION}


function VIDEO_PadFramebufferWidth(width: cint): pcuint16; inline;
begin
  VIDEO_PadFramebufferWidth := pcuint16((cuint16(width) + 15) and not 15);
end;

function VI_TVMODE(fmt, mode: cint): cint; inline;
begin
  VI_TVMODE :=  (fmt shl 2) + mode;
end; 

{$endif OGC_IMPLEMENTATION}
