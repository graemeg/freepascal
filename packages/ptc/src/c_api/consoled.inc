{ setup }
Function ptc_console_create : TPTC_CONSOLE;
Procedure ptc_console_destroy(obj : TPTC_CONSOLE);

{ console configuration }
Procedure ptc_console_configure(obj : TPTC_CONSOLE; _file : String);

{ console option string }
Function ptc_console_option(obj : TPTC_CONSOLE; _option : String) : Boolean;

{ console modes }
Function ptc_console_mode(obj : TPTC_CONSOLE; index : Integer) : TPTC_MODE;

{ console management }
Procedure ptc_console_open(obj : TPTC_CONSOLE; title : String; pages : Integer);
Procedure ptc_console_open_format(obj : TPTC_CONSOLE; title : String; format : TPTC_FORMAT; pages : Integer);
Procedure ptc_console_open_resolution(obj : TPTC_CONSOLE; title : String; width, height : Integer; format : TPTC_FORMAT; pages : Integer);
Procedure ptc_console_open_mode(obj : TPTC_CONSOLE; title : String; mode : TPTC_MODE; pages : Integer);
Procedure ptc_console_close(obj : TPTC_CONSOLE);

{ synchronization }
Procedure ptc_console_flush(obj : TPTC_CONSOLE);
Procedure ptc_console_finish(obj : TPTC_CONSOLE);
Procedure ptc_console_update(obj : TPTC_CONSOLE);
Procedure ptc_console_update_area(obj : TPTC_CONSOLE; area : TPTC_AREA);

{ keyboard input }
Function ptc_console_key(obj : TPTC_CONSOLE) : Boolean;
Procedure ptc_console_read(obj : TPTC_CONSOLE; key : TPTC_KEY);

{ copy to surface }
Procedure ptc_console_copy(obj : TPTC_CONSOLE; surface : TPTC_SURFACE);
Procedure ptc_console_copy_area(obj : TPTC_CONSOLE; surface : TPTC_SURFACE; source, destination : TPTC_AREA);

{ memory access }
Function ptc_console_lock(obj : TPTC_CONSOLE) : Pointer;
Procedure ptc_console_unlock(obj : TPTC_CONSOLE);

{ load pixels to console }
Procedure ptc_console_load(obj : TPTC_CONSOLE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE);
Procedure ptc_console_load_area(obj : TPTC_CONSOLE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE; source, destination : TPTC_AREA);

{ save console pixels }
Procedure ptc_console_save(obj : TPTC_CONSOLE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE);
Procedure ptc_console_save_area(obj : TPTC_CONSOLE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE; source, destination : TPTC_AREA);

{ clear console }
Procedure ptc_console_clear(obj : TPTC_CONSOLE);
Procedure ptc_console_clear_color(obj : TPTC_CONSOLE; color : TPTC_COLOR);
Procedure ptc_console_clear_color_area(obj : TPTC_CONSOLE; color : TPTC_COLOR; area : TPTC_AREA);

{ console palette }
Procedure ptc_console_palette_set(obj : TPTC_CONSOLE; palette : TPTC_PALETTE);
Function ptc_console_palette_get(obj : TPTC_CONSOLE) : TPTC_PALETTE;

{ console clip area }
Procedure ptc_console_clip_set(obj : TPTC_CONSOLE; area : TPTC_AREA);

{ data access }
Function ptc_console_width(obj : TPTC_CONSOLE) : Integer;
Function ptc_console_height(obj : TPTC_CONSOLE) : Integer;
Function ptc_console_pages(obj : TPTC_CONSOLE) : Integer;
Function ptc_console_pitch(obj : TPTC_CONSOLE) : Integer;
Function ptc_console_area(obj : TPTC_CONSOLE) : TPTC_AREA;
Function ptc_console_clip(obj : TPTC_CONSOLE) : TPTC_AREA;
Function ptc_console_format(obj : TPTC_CONSOLE) : TPTC_FORMAT;
Function ptc_console_name(obj : TPTC_CONSOLE) : String;
Function ptc_console_title(obj : TPTC_CONSOLE) : String;
Function ptc_console_information(obj : TPTC_CONSOLE) : String;

{ extension functions }
{#ifdef __PTC_WIN32_EXTENSIONS__
CAPI void PTCAPI ptc_console_open_window(PTC_CONSOLE object,HWND window,int pages);
CAPI void PTCAPI ptc_console_open_window_format(PTC_CONSOLE object,HWND window,PTC_FORMAT format,int pages);
CAPI void PTCAPI ptc_console_open_window_resolution(PTC_CONSOLE object,HWND window,int width,int height,PTC_FORMAT format,int pages);
CAPI void PTCAPI ptc_console_open_window_mode(PTC_CONSOLE object,HWND window,PTC_MODE mode,int pages);
CAPI HWND PTCAPI ptc_console_window(PTC_CONSOLE object);
CAPI LPDIRECTDRAW PTCAPI ptc_console_lpDD(PTC_CONSOLE object);
CAPI LPDIRECTDRAW2 PTCAPI ptc_console_lpDD2(PTC_CONSOLE object);
CAPI LPDIRECTDRAWSURFACE PTCAPI ptc_console_lpDDS(PTC_CONSOLE object);
CAPI LPDIRECTDRAWSURFACE PTCAPI ptc_console_lpDDS_primary(PTC_CONSOLE object);
CAPI LPDIRECTDRAWSURFACE PTCAPI ptc_console_lpDDS_secondary(PTC_CONSOLE object);
#endif}
