{ setup }
Function ptc_copy_create : TPTC_COPY;
Procedure ptc_copy_destroy(obj : TPTC_COPY);

{ set source and destination formats }
Procedure ptc_copy_request(obj : TPTC_COPY; source, destination : TPTC_FORMAT);

{ set source and destination palettes }
Procedure ptc_copy_palette(obj : TPTC_COPY; source, destination : TPTC_PALETTE);

{ copy pixels }
Procedure ptc_copy_copy(obj : TPTC_COPY; source_pixels : Pointer; source_x, source_y, source_width, source_height, source_pitch : Integer;
                        destination_pixels : Pointer; destination_x, destination_y, destination_width, destination_height, destination_pitch : Integer);

{ copy option string }
Function ptc_copy_option(obj : TPTC_COPY; option : String) : Boolean;
