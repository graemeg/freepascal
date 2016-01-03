{ setup }
Function ptc_surface_create(width, height : Integer; format : TPTC_FORMAT) : TPTC_SURFACE;
Procedure ptc_surface_destroy(obj : TPTC_SURFACE);

{ copy to surface }
Procedure ptc_surface_copy(obj : TPTC_SURFACE; surface : TPTC_SURFACE);
Procedure ptc_surface_copy_area(obj : TPTC_SURFACE; surface : TPTC_SURFACE; source, destination : TPTC_AREA);

{ memory access }
Function ptc_surface_lock(obj : TPTC_SURFACE) : Pointer;
Procedure ptc_surface_unlock(obj : TPTC_SURFACE);

{ load pixels to surface }
Procedure ptc_surface_load(obj : TPTC_SURFACE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE);
Procedure ptc_surface_load_area(obj : TPTC_SURFACE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE; source, destination : TPTC_AREA);

{ save surface pixels }
Procedure ptc_surface_save(obj : TPTC_SURFACE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE);
Procedure ptc_surface_save_area(obj : TPTC_SURFACE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE; source, destination : TPTC_AREA);

{ clear surface }
Procedure ptc_surface_clear(obj : TPTC_SURFACE);
Procedure ptc_surface_clear_color(obj : TPTC_SURFACE; color : TPTC_COLOR);
Procedure ptc_surface_clear_color_area(obj : TPTC_SURFACE; color : TPTC_COLOR; area : TPTC_AREA);

{ surface palette }
Procedure ptc_surface_palette_set(obj : TPTC_SURFACE; palette : TPTC_PALETTE);
Function ptc_surface_palette_get(obj : TPTC_SURFACE) : TPTC_PALETTE;

{ surface clip area }
Procedure ptc_surface_clip_set(obj : TPTC_SURFACE; area : TPTC_AREA);

{ data access }
Function ptc_surface_width(obj : TPTC_SURFACE) : Integer;
Function ptc_surface_height(obj : TPTC_SURFACE) : Integer;
Function ptc_surface_pitch(obj : TPTC_SURFACE) : Integer;
Function ptc_surface_area(obj : TPTC_SURFACE) : TPTC_AREA;
Function ptc_surface_clip(obj : TPTC_SURFACE) : TPTC_AREA;
Function ptc_surface_format(obj : TPTC_SURFACE) : TPTC_FORMAT;

{ surface option string }
Function ptc_surface_option(obj : TPTC_SURFACE; _option : String) : Boolean;
