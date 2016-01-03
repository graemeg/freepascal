{ setup }
Function ptc_palette_create : TPTC_PALETTE;
{Function ptc_palette_create_data(data : Pint32) : TPTC_PALETTE;}
Procedure ptc_palette_destroy(obj : TPTC_PALETTE);

{ memory access }
Function ptc_palette_lock(obj : TPTC_PALETTE) : Pint32;
Procedure ptc_palette_unlock(obj : TPTC_PALETTE);

{ load palette data }
Procedure ptc_palette_load(obj : TPTC_PALETTE; data : Pint32);

{ save palette data }
Procedure ptc_palette_save(obj : TPTC_PALETTE; data : Pint32);

{ get palette data }
Function ptc_palette_data(obj : TPTC_PALETTE) : Pint32;

{ operators }
Procedure ptc_palette_assign(obj, palette : TPTC_PALETTE);
Function ptc_palette_equals(obj, palette : TPTC_PALETTE) : Boolean;
