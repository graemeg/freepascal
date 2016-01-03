{$IFDEF OGC_INTERFACE}
const
  GX_FALSE = 0;
  GX_TRUE = 1;
  GX_DISABLE = 0;
  GX_ENABLE = 1;

  GX_CLIP_DISABLE = 1;
  GX_CLIP_ENABLE = 0;

  GX_FIFO_MINSIZE = ( 64 * 1024 );  (*!< Smallest usable graphics FIFO size.  *)
  GX_FIFO_HIWATERMARK = ( 16 * 1024 );  (*!< Default hi watermark for FIFO buffer control.  *)
  GX_FIFO_OBJSIZE = 128;
  GX_PERSPECTIVE = 0;
  GX_ORTHOGRAPHIC = 1;
  GX_MT_NULL = 0;
  GX_MT_XF_FLUSH = 1;
  GX_MT_DL_SAVE_CTX = 2;
  GX_XF_FLUSH_NONE = 0;
  GX_XF_FLUSH_SAFE = 1;
  GX_COLOR0 = 0;
  GX_COLOR1 = 1;
  GX_ALPHA0 = 2;
  GX_ALPHA1 = 3;
  GX_COLOR0A0 = 4;
  GX_COLOR1A1 = 5;
  GX_COLORZERO = 6;
  GX_ALPHA_BUMP = 7;
  GX_ALPHA_BUMPN = 8;
  GX_COLORNULL = $ff;

  GX_MTX2x4 = 0;
  GX_MTX3x4 = 1;
  GX_VTXFMT0 = 0;
  GX_VTXFMT1 = 1;
  GX_VTXFMT2 = 2;
  GX_VTXFMT3 = 3;
  GX_VTXFMT4 = 4;
  GX_VTXFMT5 = 5;
  GX_VTXFMT6 = 6;
  GX_VTXFMT7 = 7;
  GX_MAXVTXFMT = 8;

  GX_NONE = 0;  (*!< Input data is not used  *)

  GX_DIRECT = 1;  (*!< Input data is set direct  *)
  GX_INDEX8 = 2;  (*!< Input data is set by a 8bit index  *)
  GX_INDEX16 = 3;  (*!< Input data is set by a 16bit index  *)
  GX_U8 = 0;  (*!< Unsigned 8-bit integer  *)
  GX_S8 = 1;  (*!< Signed 8-bit integer  *)
  GX_U16 = 2;  (*!< Unsigned 16-bit integer  *)
  GX_S16 = 3;  (*!< Signed 16-bit integer  *)
  GX_F32 = 4;  (*!< 32-bit floating-point  *)
  GX_RGB565 = 0;  (*!< 16-bit RGB  *)
  GX_RGB8 = 1;  (*!< 24-bit RGB  *)
  GX_RGBX8 = 2;  (*!< 32-bit RGBX  *)
  GX_RGBA4 = 3;  (*!< 16-bit RGBA  *)
  GX_RGBA6 = 4;  (*!< 24-bit RGBA  *)
  GX_RGBA8 = 5;  (*!< 32-bit RGBA  *)
  GX_POS_XY = 0;  (*!< X,Y position  *)
  GX_POS_XYZ = 1;  (*!< X,Y,Z position  *)
  GX_NRM_XYZ = 0;  (*!< X,Y,Z normal  *)
  GX_NRM_NBT = 1;
  GX_NRM_NBT3 = 2;
  GX_CLR_RGB = 0;  (*!< RGB color  *)
  GX_CLR_RGBA = 1;  (*!< RGBA color  *)
  GX_TEX_S = 0;  (*!< One texture dimension  *)
  GX_TEX_ST = 1;  (*!< Two texture dimensions  *)
  GX_VA_PTNMTXIDX = 0;
  GX_VA_TEX0MTXIDX = 1;
  GX_VA_TEX1MTXIDX = 2;
  GX_VA_TEX2MTXIDX = 3;
  GX_VA_TEX3MTXIDX = 4;
  GX_VA_TEX4MTXIDX = 5;
  GX_VA_TEX5MTXIDX = 6;
  GX_VA_TEX6MTXIDX = 7;
  GX_VA_TEX7MTXIDX = 8;
  GX_VA_POS = 9;
  GX_VA_NRM = 10;
  GX_VA_CLR0 = 11;
  GX_VA_CLR1 = 12;
  GX_VA_TEX0 = 13;
  GX_VA_TEX1 = 14;
  GX_VA_TEX2 = 15;
  GX_VA_TEX3 = 16;
  GX_VA_TEX4 = 17;
  GX_VA_TEX5 = 18;
  GX_VA_TEX6 = 19;
  GX_VA_TEX7 = 20;
  GX_POSMTXARRAY = 21;
  GX_NRMMTXARRAY = 22;
  GX_TEXMTXARRAY = 23;
  GX_LIGHTARRAY = 24;
  GX_VA_NBT = 25;
  GX_VA_MAXATTR = 26;
  GX_VA_NULL = $ff;
  GX_POINTS = $B8;  (*!< Draws a series of points. Each vertex is a single point.  *)
  GX_LINES = $A8;  (*!< Draws a series of unconnected line segments. Each pair of vertices makes a line.  *)
  GX_LINESTRIP = $B0;  (*!< Draws a series of lines. Each vertex (besides the first) makes a line between it and the previous.  *)
  GX_TRIANGLES = $90;  (*!< Draws a series of unconnected triangles. Three vertices make a single triangle.  *)
  GX_TRIANGLESTRIP = $98;  (*!< Draws a series of triangles. Each triangle (besides the first) shares a side with the previous triangle.
												* Each vertex (besides the first two) completes a triangle.  *)
  GX_TRIANGLEFAN = $A0;  (*!< Draws a single triangle fan. The first vertex is the "centerpoint". The second and third vertex complete
												* the first triangle. Each subsequent vertex completes another triangle which shares a side with the previous
												* triangle (except the first triangle) and has the centerpoint vertex as one of the vertices.  *)
  GX_QUADS = $80;  (*!< Draws a series of unconnected quads. Every four vertices completes a quad. Internally, each quad is
												* translated into a pair of triangles.  *)
  GX_SRC_REG = 0;
  GX_SRC_VTX = 1;
  GX_LIGHT0 = $001;  (*!< Light 0  *)
  GX_LIGHT1 = $002;  (*!< Light 2  *)
  GX_LIGHT2 = $004;  (*!< Light 3  *)
  GX_LIGHT3 = $008;  (*!< Light 4  *)
  GX_LIGHT4 = $010;  (*!< Light 5  *)
  GX_LIGHT5 = $020;  (*!< Light 6  *)
  GX_LIGHT6 = $040;  (*!< Light 7  *)
  GX_LIGHT7 = $080;  (*!< Light 8  *)
  GX_MAXLIGHT = $100;  (*!< All lights  *)
  GX_LIGHTNULL = $000;  (*!< No lights  *)
  GX_DF_NONE = 0;
  GX_DF_SIGNED = 1;
  GX_DF_CLAMP = 2;
  GX_AF_SPEC = 0;  (*!< Specular computation  *)
  GX_AF_SPOT = 1;  (*!< Spot light attenuation  *)
  GX_AF_NONE = 2;  (*!< No attenuation  *)
  GX_PNMTX0 = 0;
  GX_PNMTX1 = 3;
  GX_PNMTX2 = 6;
  GX_PNMTX3 = 9;
  GX_PNMTX4 = 12;
  GX_PNMTX5 = 15;
  GX_PNMTX6 = 18;
  GX_PNMTX7 = 21;
  GX_PNMTX8 = 24;
  GX_PNMTX9 = 27;
  GX_TEXMTX0 = 30;
  GX_TEXMTX1 = 33;
  GX_TEXMTX2 = 36;
  GX_TEXMTX3 = 39;
  GX_TEXMTX4 = 42;
  GX_TEXMTX5 = 45;
  GX_TEXMTX6 = 48;
  GX_TEXMTX7 = 51;
  GX_TEXMTX8 = 54;
  GX_TEXMTX9 = 57;
  GX_IDENTITY = 60;
  GX_DTTMTX0 = 64;
  GX_DTTMTX1 = 67;
  GX_DTTMTX2 = 70;
  GX_DTTMTX3 = 73;
  GX_DTTMTX4 = 76;
  GX_DTTMTX5 = 79;
  GX_DTTMTX6 = 82;
  GX_DTTMTX7 = 85;
  GX_DTTMTX8 = 88;
  GX_DTTMTX9 = 91;
  GX_DTTMTX10 = 94;
  GX_DTTMTX11 = 97;
  GX_DTTMTX12 = 100;
  GX_DTTMTX13 = 103;
  GX_DTTMTX14 = 106;
  GX_DTTMTX15 = 109;
  GX_DTTMTX16 = 112;
  GX_DTTMTX17 = 115;
  GX_DTTMTX18 = 118;
  GX_DTTMTX19 = 121;
  GX_DTTIDENTITY = 125;
  GX_TEXCOORD0 = $0;
  GX_TEXCOORD1 = $1;
  GX_TEXCOORD2 = $2;
  GX_TEXCOORD3 = $3;
  GX_TEXCOORD4 = $4;
  GX_TEXCOORD5 = $5;
  GX_TEXCOORD6 = $6;
  GX_TEXCOORD7 = $7;
  GX_MAXCOORD = $8;
  GX_TEXCOORDNULL = $ff;
  _GX_TF_ZTF = $10;
  _GX_TF_CTF = $20;
  GX_TF_I4 = $0;
  GX_TF_I8 = $1;
  GX_TF_IA4 = $2;
  GX_TF_IA8 = $3;
  GX_TF_RGB565 = $4;
  GX_TF_RGB5A3 = $5;
  GX_TF_RGBA8 = $6;
  GX_TF_CI4 = $8;
  GX_TF_CI8 = $9;
  GX_TF_CI14 = $a;
  GX_TF_CMPR = $E;  (*!< Compressed  *)
  GX_TL_IA8 = $00;
  GX_TL_RGB565 = $01;
  GX_TL_RGB5A3 = $02;
  GX_CTF_R4 = ( $0 or _GX_TF_CTF );  (*!< For copying 4 bits from red  *)
  GX_CTF_RA4 = ( $2 or _GX_TF_CTF );  (*!< For copying 4 bits from red, 4 bits from alpha  *)
  GX_CTF_RA8 = ( $3 or _GX_TF_CTF );  (*!< For copying 8 bits from red, 8 bits from alpha  *)
  GX_CTF_YUVA8 = ( $6 or _GX_TF_CTF );
  GX_CTF_A8 = ( $7 or _GX_TF_CTF );  (*!< For copying 8 bits from alpha  *)
  GX_CTF_R8 = ( $8 or _GX_TF_CTF );  (*!< For copying 8 bits from red  *)
  GX_CTF_G8 = ( $9 or _GX_TF_CTF );  (*!< For copying 8 bits from green  *)
  GX_CTF_B8 = ( $A or _GX_TF_CTF );  (*!< For copying 8 bits from blue  *)
  GX_CTF_RG8 = ( $B or _GX_TF_CTF );  (*!< For copying 8 bits from red, 8 bits from green  *)
  GX_CTF_GB8 = ( $C or _GX_TF_CTF );  (*!< For copying 8 bits from green, 8 bits from blue  *)
  GX_TF_Z8 = ( $1 or _GX_TF_ZTF );  (*!< For texture copy, specifies upper 8 bits of Z  *)
  GX_TF_Z16 = ( $3 or _GX_TF_ZTF );  (*!< For texture copy, specifies upper 16 bits of Z  *)
  GX_TF_Z24X8 = ( $6 or _GX_TF_ZTF );  (*!< For texture copy, copies 24 Z bits and 0xFF  *)
  GX_CTF_Z4 = ( $0 or _GX_TF_ZTF or _GX_TF_CTF );  (*!< For copying 4 upper bits from Z  *)
  GX_CTF_Z8M = ( $9 or _GX_TF_ZTF or _GX_TF_CTF );  (*!< For copying the middle 8 bits of Z  *)
  GX_CTF_Z8L = ( $A or _GX_TF_ZTF or _GX_TF_CTF );  (*!< For copying the lower 8 bits of Z  *)
  GX_CTF_Z16L = ( $C or _GX_TF_ZTF or _GX_TF_CTF );  (*!< For copying the lower 16 bits of Z  *)
  GX_TF_A8 = GX_CTF_A8;
  GX_TLUT_16 = 1;  // number of 16 entry blocks.
  GX_TLUT_64 = 4;
  GX_TLUT_128 = 8;
  GX_TLUT_256 = 16;
  GX_TLUT_512 = 32;
  GX_TLUT_1K = 64;
  GX_TLUT_2K = 128;
  GX_TLUT_4K = 256;
  GX_TLUT_8K = 512;
  GX_TLUT_16K = 1024;
  GX_ZT_DISABLE = 0;
  GX_ZT_ADD = 1;  (*!< Add a Z texel to reference Z  *)
  GX_ZT_REPLACE = 2;  (*!< Replace reference Z with Z texel  *)
  GX_MAX_ZTEXOP = 3;
  GX_TG_MTX3x4 = 0;  (*!< 2x4 matrix multiply on the input attribute and generate S,T texture coordinates.  *)
  GX_TG_MTX2x4 = 1;  (*!< 3x4 matrix multiply on the input attribute and generate S,T,Q coordinates; S,T are then divided
										* by Q to produce the actual 2D texture coordinates.  *)
  GX_TG_BUMP0 = 2;  (*!< Use light 0 in the bump map calculation.  *)
  GX_TG_BUMP1 = 3;  (*!< Use light 1 in the bump map calculation.  *)
  GX_TG_BUMP2 = 4;  (*!< Use light 2 in the bump map calculation.  *)
  GX_TG_BUMP3 = 5;  (*!< Use light 3 in the bump map calculation.  *)
  GX_TG_BUMP4 = 6;  (*!< Use light 4 in the bump map calculation.  *)
  GX_TG_BUMP5 = 7;  (*!< Use light 5 in the bump map calculation.  *)
  GX_TG_BUMP6 = 8;  (*!< Use light 6 in the bump map calculation.  *)
  GX_TG_BUMP7 = 9;  (*!< Use light 7 in the bump map calculation.  *)
  GX_TG_SRTG = 10;  (*!< Coordinates generated from vertex lighting results; one of the color channel results is converted
										* into texture coordinates.  *)
  GX_TG_POS = 0;
  GX_TG_NRM = 1;
  GX_TG_BINRM = 2;
  GX_TG_TANGENT = 3;
  GX_TG_TEX0 = 4;
  GX_TG_TEX1 = 5;
  GX_TG_TEX2 = 6;
  GX_TG_TEX3 = 7;
  GX_TG_TEX4 = 8;
  GX_TG_TEX5 = 9;
  GX_TG_TEX6 = 10;
  GX_TG_TEX7 = 11;
  GX_TG_TEXCOORD0 = 12;
  GX_TG_TEXCOORD1 = 13;
  GX_TG_TEXCOORD2 = 14;
  GX_TG_TEXCOORD3 = 15;
  GX_TG_TEXCOORD4 = 16;
  GX_TG_TEXCOORD5 = 17;
  GX_TG_TEXCOORD6 = 18;
  GX_TG_COLOR0 = 19;
  GX_TG_COLOR1 = 20;
  GX_NEVER = 0;
  GX_LESS = 1;
  GX_EQUAL = 2;
  GX_LEQUAL = 3;
  GX_GREATER = 4;
  GX_NEQUAL = 5;
  GX_GEQUAL = 6;
  GX_ALWAYS = 7;
  GX_CLAMP = 0;
  GX_REPEAT = 1;
  GX_MIRROR = 2;
  GX_MAXTEXWRAPMODE = 3;

  GX_BM_NONE = 0;  (*!< Write input directly to EFB  *)
  GX_BM_BLEND = 1;  (*!< Blend using blending equation  *)
  GX_BM_LOGIC = 2;  (*!< Blend using bitwise operation  *)
  GX_BM_SUBTRACT = 3;  (*!< Input subtracts from existing pixel  *)
  GX_MAX_BLENDMODE = 4;

  GX_BL_ZERO = 0;  (*!< 0.0  *)
  GX_BL_ONE = 1;  (*!< 1.0  *)
  GX_BL_SRCCLR = 2;  (*!< source color  *)
  GX_BL_INVSRCCLR = 3;  (*!< 1.0 - (source color)  *)
  GX_BL_SRCALPHA = 4;  (*!< source alpha  *)
  GX_BL_INVSRCALPHA = 5;  (*!< 1.0 - (source alpha)  *)
  GX_BL_DSTALPHA = 6;  (*!< framebuffer alpha  *)
  GX_BL_INVDSTALPHA = 7;  (*!< 1.0 - (FB alpha)  *)
  GX_BL_DSTCLR = GX_BL_SRCCLR;
  GX_BL_INVDSTCLR = GX_BL_INVSRCCLR;


  GX_LO_CLEAR = 0;  (*!< 0  *)
  GX_LO_AND = 1;  (*!< src & dst  *)
  GX_LO_REVAND = 2;  (*!< src & ~dst  *)
  GX_LO_COPY = 3;  (*!< src  *)
  GX_LO_INVAND = 4;  (*!< ~src & dst  *)
  GX_LO_NOOP = 5;  (*!< dst  *)
  GX_LO_XOR = 6;  (*!< src ^ dst  *)
  GX_LO_OR = 7;  (*!< src | dst  *)
  GX_LO_NOR = 8;  (*!< ~(src | dst)  *)
  GX_LO_EQUIV = 9;  (*!< ~(src ^ dst)  *)
  GX_LO_INV = 10;  (*!< ~dst  *)
  GX_LO_REVOR = 11;  (*!< src | ~dst  *)
  GX_LO_INVCOPY = 12;  (*!< ~src  *)
  GX_LO_INVOR = 13;  (*!< ~src | dst  *)
  GX_LO_NAND = 14;  (*!< ~(src & dst)  *)
  GX_LO_SET = 15;  (*!< 1  *)

  GX_TO_ZERO = 0;
  GX_TO_SIXTEENTH = 1;
  GX_TO_EIGHTH = 2;
  GX_TO_FOURTH = 3;
  GX_TO_HALF = 4;
  GX_TO_ONE = 5;
  GX_MAX_TEXOFFSET = 6;

  GX_MODULATE = 0;  (*!< <i>Cv</i>=<i>CrCt</i>; <i>Av</i>=<i>ArAt</i>  *)
  GX_DECAL = 1;  (*!< <i>Cv</i>=(1-<i>At</i>)<i>Cr</i> + <i>AtCt</i>; <i>Av</i>=<i>Ar</i>  *)
  GX_BLEND = 2;  (*!< <i>Cv=(1-<i>Ct</i>)<i>Cr</i> + <i>Ct</i>; <i>Av</i>=<i>AtAr</i>  *)
  GX_REPLACE = 3;  (*!< <i>Cv=<i>Ct</i>; <i>Ar=<i>At</i>  *)
  GX_PASSCLR = 4;  (*!< <i>Cv=<i>Cr</i>; <i>Av=<i>Ar</i>  *)

  GX_CC_CPREV = 0;  (*!< Use the color value from previous TEV stage  *)
  GX_CC_APREV = 1;  (*!< Use the alpha value from previous TEV stage  *)
  GX_CC_C0 = 2;  (*!< Use the color value from the color/output register 0  *)
  GX_CC_A0 = 3;  (*!< Use the alpha value from the color/output register 0  *)
  GX_CC_C1 = 4;  (*!< Use the color value from the color/output register 1  *)
  GX_CC_A1 = 5;  (*!< Use the alpha value from the color/output register 1  *)
  GX_CC_C2 = 6;  (*!< Use the color value from the color/output register 2  *)
  GX_CC_A2 = 7;  (*!< Use the alpha value from the color/output register 2  *)
  GX_CC_TEXC = 8;  (*!< Use the color value from texture  *)
  GX_CC_TEXA = 9;  (*!< Use the alpha value from texture  *)
  GX_CC_RASC = 10;  (*!< Use the color value from rasterizer  *)
  GX_CC_RASA = 11;  (*!< Use the alpha value from rasterizer  *)
  GX_CC_ONE = 12;
  GX_CC_HALF = 13;
  GX_CC_KONST = 14;
  GX_CC_ZERO = 15;  (*!< Use to pass zero value  *)

  GX_CA_APREV = 0;  (*!< Use the alpha value from previous TEV stage  *)
  GX_CA_A0 = 1;  (*!< Use the alpha value from the color/output register 0  *)
  GX_CA_A1 = 2;  (*!< Use the alpha value from the color/output register 1  *)
  GX_CA_A2 = 3;  (*!< Use the alpha value from the color/output register 2  *)
  GX_CA_TEXA = 4;  (*!< Use the alpha value from texture  *)
  GX_CA_RASA = 5;  (*!< Use the alpha value from rasterizer  *)
  GX_CA_KONST = 6;
  GX_CA_ZERO = 7;  (*!< Use to pass zero value  *)

  GX_TEVSTAGE0 = 0;
  GX_TEVSTAGE1 = 1;
  GX_TEVSTAGE2 = 2;
  GX_TEVSTAGE3 = 3;
  GX_TEVSTAGE4 = 4;
  GX_TEVSTAGE5 = 5;
  GX_TEVSTAGE6 = 6;
  GX_TEVSTAGE7 = 7;
  GX_TEVSTAGE8 = 8;
  GX_TEVSTAGE9 = 9;
  GX_TEVSTAGE10 = 10;
  GX_TEVSTAGE11 = 11;
  GX_TEVSTAGE12 = 12;
  GX_TEVSTAGE13 = 13;
  GX_TEVSTAGE14 = 14;
  GX_TEVSTAGE15 = 15;
  GX_MAX_TEVSTAGE = 16;

  GX_TEV_ADD = 0;
  GX_TEV_SUB = 1;
  GX_TEV_COMP_R8_GT = 8;
  GX_TEV_COMP_R8_EQ = 9;
  GX_TEV_COMP_GR16_GT = 10;
  GX_TEV_COMP_GR16_EQ = 11;
  GX_TEV_COMP_BGR24_GT = 12;
  GX_TEV_COMP_BGR24_EQ = 13;
  GX_TEV_COMP_RGB8_GT = 14;
  GX_TEV_COMP_RGB8_EQ = 15;
  GX_TEV_COMP_A8_GT = GX_TEV_COMP_RGB8_GT;  // for alpha channel

  GX_TB_ZERO = 0;
  GX_TB_ADDHALF = 1;
  GX_TB_SUBHALF = 2;
  GX_MAX_TEVBIAS = 3;

  GX_TC_LINEAR = 0;
  GX_TC_GE = 1;
  GX_TC_EQ = 2;
  GX_TC_LE = 3;
  GX_MAX_TEVCLAMPMODE = 4;

  GX_CS_SCALE_1 = 0;
  GX_CS_SCALE_2 = 1;
  GX_CS_SCALE_4 = 2;
  GX_CS_DIVIDE_2 = 3;
  GX_MAX_TEVSCALE = 4;

  GX_TEVPREV = 0;  (*!< Default register for passing results from one stage to another.  *)
  GX_TEVREG0 = 1;
  GX_TEVREG1 = 2;
  GX_TEVREG2 = 3;
  GX_MAX_TEVREG = 4;

  GX_CULL_NONE = 0;  (*!< Do not cull any primitives.  *)
  GX_CULL_FRONT = 1;  (*!< Cull front-facing primitives.  *)
  GX_CULL_BACK = 2;  (*!< Cull back-facing primitives.  *)
  GX_CULL_ALL = 3;  (*!< Cull all primitives.  *)

  GX_TEXMAP0 = 0;  (*!< Texture map slot 0  *)
  GX_TEXMAP1 = 1;  (*!< Texture map slot 1  *)
  GX_TEXMAP2 = 2;  (*!< Texture map slot 2  *)
  GX_TEXMAP3 = 3;  (*!< Texture map slot 3  *)
  GX_TEXMAP4 = 4;  (*!< Texture map slot 4  *)
  GX_TEXMAP5 = 5;  (*!< Texture map slot 5  *)
  GX_TEXMAP6 = 6;  (*!< Texture map slot 6  *)
  GX_TEXMAP7 = 7;  (*!< Texture map slot 7  *)
  GX_MAX_TEXMAP = 8;
  GX_TEXMAP_NULL = $ff;  (*!< No texmap  *)
  GX_TEXMAP_DISABLE = $100;  (*!< Disable texmap lookup for this texmap slot (use bitwise OR with a texture map slot).  *)

  GX_AOP_AND = 0;
  GX_AOP_OR = 1;
  GX_AOP_XOR = 2;
  GX_AOP_XNOR = 3;
  GX_MAX_ALPHAOP = 4;

  GX_KCOLOR0 = 0;  (*!< Constant register 0  *)
  GX_KCOLOR1 = 1;  (*!< Constant register 1  *)
  GX_KCOLOR2 = 2;  (*!< Constant register 2  *)
  GX_KCOLOR3 = 3;  (*!< Constant register 3  *)
  GX_KCOLOR_MAX = 4;

  GX_TEV_KCSEL_1 = $00;  (*!< constant 1.0  *)
  GX_TEV_KCSEL_7_8 = $01;  (*!< constant 7/8  *)
  GX_TEV_KCSEL_3_4 = $02;  (*!< constant 3/4  *)
  GX_TEV_KCSEL_5_8 = $03;  (*!< constant 5/8  *)
  GX_TEV_KCSEL_1_2 = $04;  (*!< constant 1/2  *)
  GX_TEV_KCSEL_3_8 = $05;  (*!< constant 3/8  *)
  GX_TEV_KCSEL_1_4 = $06;  (*!< constant 1/4  *)
  GX_TEV_KCSEL_1_8 = $07;  (*!< constant 1/8  *)
  GX_TEV_KCSEL_K0 = $0C;  (*!< K0[RGB] register  *)
  GX_TEV_KCSEL_K1 = $0D;  (*!< K1[RGB] register  *)
  GX_TEV_KCSEL_K2 = $0E;  (*!< K2[RGB] register  *)
  GX_TEV_KCSEL_K3 = $0F;  (*!< K3[RGB] register  *)
  GX_TEV_KCSEL_K0_R = $10;  (*!< K0[RRR] register  *)
  GX_TEV_KCSEL_K1_R = $11;  (*!< K1[RRR] register  *)
  GX_TEV_KCSEL_K2_R = $12;  (*!< K2[RRR] register  *)
  GX_TEV_KCSEL_K3_R = $13;  (*!< K3[RRR] register  *)
  GX_TEV_KCSEL_K0_G = $14;  (*!< K0[GGG] register  *)
  GX_TEV_KCSEL_K1_G = $15;  (*!< K1[GGG] register  *)
  GX_TEV_KCSEL_K2_G = $16;  (*!< K2[GGG] register  *)
  GX_TEV_KCSEL_K3_G = $17;  (*!< K3[GGG] register  *)
  GX_TEV_KCSEL_K0_B = $18;  (*!< K0[BBB] register  *)
  GX_TEV_KCSEL_K1_B = $19;  (*!< K1[BBB] register  *)
  GX_TEV_KCSEL_K2_B = $1A;  (*!< K2[BBB] register  *)
  GX_TEV_KCSEL_K3_B = $1B;  (*!< K3[RBB] register  *)
  GX_TEV_KCSEL_K0_A = $1C;  (*!< K0[AAA] register  *)
  GX_TEV_KCSEL_K1_A = $1D;  (*!< K1[AAA] register  *)
  GX_TEV_KCSEL_K2_A = $1E;  (*!< K2[AAA] register  *)
  GX_TEV_KCSEL_K3_A = $1F;  (*!< K3[AAA] register  *)

  GX_TEV_KASEL_1 = $00;  (*!< constant 1.0  *)
  GX_TEV_KASEL_7_8 = $01;  (*!< constant 7/8  *)
  GX_TEV_KASEL_3_4 = $02;  (*!< constant 3/4  *)
  GX_TEV_KASEL_5_8 = $03;  (*!< constant 5/8  *)
  GX_TEV_KASEL_1_2 = $04;  (*!< constant 1/2  *)
  GX_TEV_KASEL_3_8 = $05;  (*!< constant 3/8  *)
  GX_TEV_KASEL_1_4 = $06;  (*!< constant 1/4  *)
  GX_TEV_KASEL_1_8 = $07;  (*!< constant 1/8  *)
  GX_TEV_KASEL_K0_R = $10;  (*!< K0[R] register  *)
  GX_TEV_KASEL_K1_R = $11;  (*!< K1[R] register  *)
  GX_TEV_KASEL_K2_R = $12;  (*!< K2[R] register  *)
  GX_TEV_KASEL_K3_R = $13;  (*!< K3[R] register  *)
  GX_TEV_KASEL_K0_G = $14;  (*!< K0[G] register  *)
  GX_TEV_KASEL_K1_G = $15;  (*!< K1[G] register  *)
  GX_TEV_KASEL_K2_G = $16;  (*!< K2[G] register  *)
  GX_TEV_KASEL_K3_G = $17;  (*!< K3[G] register  *)
  GX_TEV_KASEL_K0_B = $18;  (*!< K0[B] register  *)
  GX_TEV_KASEL_K1_B = $19;  (*!< K1[B] register  *)
  GX_TEV_KASEL_K2_B = $1A;  (*!< K2[B] register  *)
  GX_TEV_KASEL_K3_B = $1B;  (*!< K3[B] register  *)
  GX_TEV_KASEL_K0_A = $1C;  (*!< K0[A] register  *)
  GX_TEV_KASEL_K1_A = $1D;  (*!< K1[A] register  *)
  GX_TEV_KASEL_K2_A = $1E;  (*!< K2[A] register  *)
  GX_TEV_KASEL_K3_A = $1F;  (*!< K3[A] register  *)

  GX_TEV_SWAP0 = 0;
  GX_TEV_SWAP1 = 1;
  GX_TEV_SWAP2 = 2;
  GX_TEV_SWAP3 = 3;
  GX_MAX_TEVSWAP = 4;

  GX_CH_RED = 0;
  GX_CH_GREEN = 1;
  GX_CH_BLUE = 2;
  GX_CH_ALPHA = 3;

  GX_INDTEXSTAGE0 = 0;
  GX_INDTEXSTAGE1 = 1;
  GX_INDTEXSTAGE2 = 2;
  GX_INDTEXSTAGE3 = 3;
  GX_MAX_INDTEXSTAGE = 4;

  GX_ITF_8 = 0;
  GX_ITF_5 = 1;
  GX_ITF_4 = 2;
  GX_ITF_3 = 3;
  GX_MAX_ITFORMAT = 4;

  GX_ITB_NONE = 0;
  GX_ITB_S = 1;
  GX_ITB_T = 2;
  GX_ITB_ST = 3;
  GX_ITB_U = 4;
  GX_ITB_SU = 5;
  GX_ITB_TU = 6;
  GX_ITB_STU = 7;
  GX_MAX_ITBIAS = 8;

  GX_ITM_OFF = 0;  (*!< Specifies a matrix of all zeroes.  *)
  GX_ITM_0 = 1;  (*!< Specifies indirect matrix 0, indirect scale 0.  *)
  GX_ITM_1 = 2;  (*!< Specifies indirect matrix 1, indirect scale 1.  *)
  GX_ITM_2 = 3;  (*!< Specifies indirect matrix 2, indirect scale 2.  *)
  GX_ITM_S0 = 5;  (*!< Specifies dynamic S-type matrix, indirect scale 0.  *)
  GX_ITM_S1 = 6;  (*!< Specifies dynamic S-type matrix, indirect scale 1.  *)
  GX_ITM_S2 = 7;  (*!< Specifies dynamic S-type matrix, indirect scale 2.  *)
  GX_ITM_T0 = 9;  (*!< Specifies dynamic T-type matrix, indirect scale 0.  *)
  GX_ITM_T1 = 10;  (*!< Specifies dynamic T-type matrix, indirect scale 1.  *)
  GX_ITM_T2 = 11;  (*!< Specifies dynamic T-type matrix, indirect scale 2.  *)

  GX_ITW_OFF = 0;
  GX_ITW_256 = 1;
  GX_ITW_128 = 2;
  GX_ITW_64 = 3;
  GX_ITW_32 = 4;
  GX_ITW_16 = 5;
  GX_ITW_0 = 6;
  GX_MAX_ITWRAP = 7;

  GX_ITBA_OFF = 0;
  GX_ITBA_S = 1;
  GX_ITBA_T = 2;
  GX_ITBA_U = 3;
  GX_MAX_ITBALPHA = 4;

  GX_ITS_1 = 0;
  GX_ITS_2 = 1;
  GX_ITS_4 = 2;
  GX_ITS_8 = 3;
  GX_ITS_16 = 4;
  GX_ITS_32 = 5;
  GX_ITS_64 = 6;
  GX_ITS_128 = 7;
  GX_ITS_256 = 8;
  GX_MAX_ITSCALE = 9;

  GX_FOG_NONE = 0;
  GX_FOG_PERSP_LIN = 2;
  GX_FOG_PERSP_EXP = 4;
  GX_FOG_PERSP_EXP2 = 5;
  GX_FOG_PERSP_REVEXP = 6;
  GX_FOG_PERSP_REVEXP2 = 7;
  GX_FOG_ORTHO_LIN = 10;
  GX_FOG_ORTHO_EXP = 12;
  GX_FOG_ORTHO_EXP2 = 13;
  GX_FOG_ORTHO_REVEXP = 14;
  GX_FOG_ORTHO_REVEXP2 = 15;
  GX_FOG_LIN = GX_FOG_PERSP_LIN;
  GX_FOG_EXP = GX_FOG_PERSP_EXP;
  GX_FOG_EXP2 = GX_FOG_PERSP_EXP2;
  GX_FOG_REVEXP = GX_FOG_PERSP_REVEXP;
  GX_FOG_REVEXP2 = GX_FOG_PERSP_REVEXP2;

  GX_PF_RGB8_Z24 = 0;
  GX_PF_RGBA6_Z24 = 1;
  GX_PF_RGB565_Z16 = 2;
  GX_PF_Z24 = 3;
  GX_PF_Y8 = 4;
  GX_PF_U8 = 5;
  GX_PF_V8 = 6;
  GX_PF_YUV420 = 7;

  GX_ZC_LINEAR = 0;
  GX_ZC_NEAR = 1;
  GX_ZC_MID = 2;
  GX_ZC_FAR = 3;

  GX_CLAMP_NONE = 0;
  GX_CLAMP_TOP = 1;
  GX_CLAMP_BOTTOM = 2;

  GX_GM_1_0 = 0;
  GX_GM_1_7 = 1;
  GX_GM_2_2 = 2;

  GX_COPY_PROGRESSIVE = 0;
  GX_COPY_INTLC_EVEN = 2;
  GX_COPY_INTLC_ODD = 3;

  GX_READ_00 = 0;  (*!< Always read 0x00.  *)
  GX_READ_FF = 1;  (*!< Always read 0xFF.  *)
  GX_READ_NONE = 2;  (*!< Always read the real alpha value.  *)

  GX_TEXCACHE_32K = 0;
  GX_TEXCACHE_128K = 1;
  GX_TEXCACHE_512K = 2;
  GX_TEXCACHE_NONE = 3;

  GX_DA_OFF = 0;
  GX_DA_GENTLE = 1;
  GX_DA_MEDIUM = 2;
  GX_DA_STEEP = 3;

  GX_SP_OFF = 0;
  GX_SP_FLAT = 1;
  GX_SP_COS = 2;
  GX_SP_COS2 = 3;
  GX_SP_SHARP = 4;
  GX_SP_RING1 = 5;
  GX_SP_RING2 = 6;

  GX_NEAR = 0;  (*!< Point sampling, no mipmap  *)
  GX_LINEAR = 1;  (*!< Bilinear filtering, no mipmap  *)
  GX_NEAR_MIP_NEAR = 2;  (*!< Point sampling, discrete mipmap  *)
  GX_LIN_MIP_NEAR = 3;  (*!< Bilinear filtering, discrete mipmap  *)
  GX_NEAR_MIP_LIN = 4;  (*!< Point sampling, linear mipmap  *)
  GX_LIN_MIP_LIN = 5;  (*!< Trilinear filtering  *)

  GX_ANISO_1 = 0;
  GX_ANISO_2 = 1;
  GX_ANISO_4 = 2;
  GX_MAX_ANISOTROPY = 3;

  GX_VC_POS = 0;
  GX_VC_NRM = 1;
  GX_VC_CLR0 = 2;
  GX_VC_CLR1 = 3;
  GX_VC_TEX0 = 4;
  GX_VC_TEX1 = 5;
  GX_VC_TEX2 = 6;
  GX_VC_TEX3 = 7;
  GX_VC_TEX4 = 8;
  GX_VC_TEX5 = 9;
  GX_VC_TEX6 = 10;
  GX_VC_TEX7 = 11;
  GX_VC_ALL = 15;

  GX_PERF0_VERTICES = 0;  (*!< Number of vertices processed by the GP.  *)
  GX_PERF0_CLIP_VTX = 1;  (*!< Number of vertices that were clipped by the GP.  *)
  GX_PERF0_CLIP_CLKS = 2;  (*!< Number of GP clocks spent clipping.  *)
  GX_PERF0_XF_WAIT_IN = 3;  (*!< Number of cycles the XF is waiting on input. If the XF is waiting a large percentage
													* of the total time, it may indicate that the CPU is not supplying data fast enough to
													* keep the GP busy.  *)
  GX_PERF0_XF_WAIT_OUT = 4;  (*!< Number of cycles the XF waits to send its output to the rest of the GP pipeline. If
													* the XF cannot output, it may indicate that the GP is currently fill-rate limited.  *)
  GX_PERF0_XF_XFRM_CLKS = 5;  (*!< Number of cycles the transform engine is busy.  *)
  GX_PERF0_XF_LIT_CLKS = 6;  (*!< Number of cycles the lighting engine is busy.  *)
  GX_PERF0_XF_BOT_CLKS = 7;  (*!< Number of cycles the bottom of the pipe (result combiner) is busy.  *)
  GX_PERF0_XF_REGLD_CLKS = 8;  (*!< Number of cycles are spent loading XF state registers.  *)
  GX_PERF0_XF_REGRD_CLKS = 9;  (*!< Number of cycles the XF reads the state registers.  *)
  GX_PERF0_CLIP_RATIO = 10;
  GX_PERF0_TRIANGLES = 11;  (*!< Number of triangles.  *)
  GX_PERF0_TRIANGLES_CULLED = 12;  (*!< Number of triangles that <i>failed</i> the front-face/back-face culling test.  *)
  GX_PERF0_TRIANGLES_PASSED = 13;  (*!< Number of triangles that <i>passed</i> the front-face/back-face culling test.  *)
  GX_PERF0_TRIANGLES_SCISSORED = 14;  (*!< Number of triangles that are scissored.  *)
  GX_PERF0_TRIANGLES_0TEX = 15;
  GX_PERF0_TRIANGLES_1TEX = 16;
  GX_PERF0_TRIANGLES_2TEX = 17;
  GX_PERF0_TRIANGLES_3TEX = 18;
  GX_PERF0_TRIANGLES_4TEX = 19;
  GX_PERF0_TRIANGLES_5TEX = 20;
  GX_PERF0_TRIANGLES_6TEX = 21;
  GX_PERF0_TRIANGLES_7TEX = 22;
  GX_PERF0_TRIANGLES_8TEX = 23;
  GX_PERF0_TRIANGLES_0CLR = 24;
  GX_PERF0_TRIANGLES_1CLR = 25;
  GX_PERF0_TRIANGLES_2CLR = 26;
  GX_PERF0_QUAD_0CVG = 27;  (*!< Number of quads having zero coverage.  *)
  GX_PERF0_QUAD_NON0CVG = 28;  (*!< Number of quads having coverage greater than zero.  *)
  GX_PERF0_QUAD_1CVG = 29;  (*!< Number of quads with 1 pixel coverage.  *)
  GX_PERF0_QUAD_2CVG = 30;  (*!< Number of quads with 2 pixel coverage.  *)
  GX_PERF0_QUAD_3CVG = 31;  (*!< Number of quads with 3 pixel coverage.  *)
  GX_PERF0_QUAD_4CVG = 32;  (*!< Number of quads with 4 pixel coverage.  *)
  GX_PERF0_AVG_QUAD_CNT = 33;  (*!< Average quad count; average based on what is unknown  *)
  GX_PERF0_CLOCKS = 34;  (*!< Number of GP clocks that have elapsed since the previous call to GX_ReadGP0Metric().  *)
  GX_PERF0_NONE = 35;  (*!< Disables performance measurement for perf0 and resets the counter.  *)

  GX_PERF1_TEXELS = 0;  (*!< Number of texels processed by the GP.  *)
  GX_PERF1_TX_IDLE = 1;  (*!< Number of clocks that the texture unit (TX) is idle.  *)
  GX_PERF1_TX_REGS = 2;  (*!< Number of GP clocks spent writing to state registers in the TX unit.  *)
  GX_PERF1_TX_MEMSTALL = 3;  (*!< Number of GP clocks the TX unit is stalled waiting for main memory.  *)
  GX_PERF1_TC_CHECK1_2 = 4;
  GX_PERF1_TC_CHECK3_4 = 5;
  GX_PERF1_TC_CHECK5_6 = 6;
  GX_PERF1_TC_CHECK7_8 = 7;
  GX_PERF1_TC_MISS = 8;  (*!< Number of texture cache misses in total?  *)
  GX_PERF1_VC_ELEMQ_FULL = 9;
  GX_PERF1_VC_MISSQ_FULL = 10;
  GX_PERF1_VC_MEMREQ_FULL = 11;
  GX_PERF1_VC_STATUS7 = 12;
  GX_PERF1_VC_MISSREP_FULL = 13;
  GX_PERF1_VC_STREAMBUF_LOW = 14;
  GX_PERF1_VC_ALL_STALLS = 15;
  GX_PERF1_VERTICES = 16;  (*!< Number of vertices processed by the GP.  *)
  GX_PERF1_FIFO_REQ = 17;  (*!< Number of lines (32B) read from the GP FIFO.  *)
  GX_PERF1_CALL_REQ = 18;  (*!< Number of lines (32B) read from called display lists.  *)
  GX_PERF1_VC_MISS_REQ = 19;  (*!< Number vertex cache miss request. Each miss requests a 32B transfer from main memory.  *)
  GX_PERF1_CP_ALL_REQ = 20;  (*!< Counts all requests (32B/request) from the GP Command Processor (CP). It should be equal to
													* the sum of counts returned by <tt>GX_PERF1_FIFO_REQ</tt>, <tt>GX_PERF1_CALL_REQ</tt>, and <tt>GX_PERF1_VC_MISS_REQ</tt>.  *)
  GX_PERF1_CLOCKS = 21;  (*!< Number of GP clocks that have elapsed since the last call to GX_ReadGP1Metric().  *)
  GX_PERF1_NONE = 22;  (*!< Disables performance measurement for perf1 and resets the counter.  *)

  GX_TLUT0 = 0;
  GX_TLUT1 = 1;
  GX_TLUT2 = 2;
  GX_TLUT3 = 3;
  GX_TLUT4 = 4;
  GX_TLUT5 = 5;
  GX_TLUT6 = 6;
  GX_TLUT7 = 7;
  GX_TLUT8 = 8;
  GX_TLUT9 = 9;
  GX_TLUT10 = 10;
  GX_TLUT11 = 11;
  GX_TLUT12 = 12;
  GX_TLUT13 = 13;
  GX_TLUT14 = 14;
  GX_TLUT15 = 15;
  GX_BIGTLUT0 = 16;
  GX_BIGTLUT1 = 17;
  GX_BIGTLUT2 = 18;
  GX_BIGTLUT3 = 19;

  GX_MAX_VTXDESC = GX_VA_MAXATTR;
  GX_MAX_VTXDESC_LISTSIZE = ( GX_VA_MAXATTR + 1 );
  GX_MAX_VTXATTRFMT = GX_VA_MAXATTR;
  GX_MAX_VTXATTRFMT_LISTSIZE = ( GX_VA_MAXATTR + 1 );
  GX_MAX_Z24 = $00ffffff;

type
  _wgpipe = record
  case Integer of
    0:(U8 : cuint8;);
    1:(S8 : cint8;);
    2:(U16 : cuint16;);
    3:(S16 : cint16;);
    4:(U32 : cuint32;);
    5:(S32 : cint32;);
    6:(F32 : cfloat;);
  end;
  TWGPipe = _wgpipe;
  PWGPipe = ^TWGPipe;

  _gx_color = record
    r : cuint8;  (*!< Red color component.  *)
    g : cuint8;  (*!< Green color component.  *)
    b : cuint8;  (*!< Blue alpha component.  *)
    a : cuint8;  (*!< Alpha component. If a function does not use the alpha value, it is safely ignored.  *)
  end;
  GXColor = _gx_color;
  PGXColor = ^GXColor;

  _gx_colors10 = record
    r : cint16;  (*!< Red color component.  *)
    g : cint16;  (*!< Green color component.  *)
    b : cint16;  (*!< Blue color component.  *)
    a : cint16;  (*!< Alpha component. If a function does not use the alpha value, it is safely ignored.  *)
  end;
  GXColorS10 = _gx_colors10;
  PGXColorS10 = ^GXColorS10;

  _gx_texobj = record
    val : array [0..7] of cuint32;
  end;
  GXTexObj = _gx_texobj;
  PGXTexObj = ^GXTexObj;

  _gx_tlutobj = record
    val : array [0..2] of cuint32;
  end;
  GXTlutObj = _gx_tlutobj;
  PGXTlutObj = ^GXTlutObj;


  _gx_texreg = record
    val : array [0..3] of cuint32;
  end;
  GXTexRegion = _gx_texreg;
  PGXTexRegion = ^GXTexRegion;

  _gx_tlutreg = record
    val : array [0..3] of cuint32;
  end;
  GXTlutRegion = _gx_tlutreg;
  PGXTlutRegion = ^GXTlutRegion;

  _gx_litobj = record
    val : array [0..15] of cuint32;
  end;
  GXLightObj = _gx_litobj;
  PGXLightObj = ^GXLightObj;

  _vtx = record
    x : f32;
    y : f32;
    z : f32;
    s : cuint16;
    t : cuint16;
    rgba : cuint32;
  end;
  Vtx = _vtx;
  PVtx = ^Vtx;

  GXVtxDesc = record
    attr : cuint8;  (*!< \ref vtxattr for this element.  *)
    type_ : cuint8;  (*!< \ref vtxattrin for this element.  *)
  end;
  PGXVtxDesc = ^GXVtxDesc;
  
  GXVtxAttrFmt = record
    vtxattr : cuint32;  (*!< \ref vtxattr for this element.  *)
    comptype : cuint32;  (*!< \ref comptype for this element.  *)
    compsize : cuint32;  (*!< \ref compsize for this element.  *)
    frac : cuint32;  (*!< Number of fractional bits for a fixed-point number.  *)
  end;
  PGXVtxAttrFmt = ^GXVtxAttrFmt;


  GXFifoObj = record
    pad : array [0..GX_FIFO_OBJSIZE-1] of cuint8;
  end;
  PGXFifoObj = ^GXFifoObj;


  GXTexReg = record
    dummy : array [0..3] of cuint8;
  end;
  PGXTexReg = ^GXTexReg;

  GXFogAdjTbl = record
    r : array [0..9] of cuint16;  (*!< u4.8 format range parameter.  *)
  end;
  PGXFogAdjTbl = ^GXFogAdjTbl;

type
  GXBreakPtCallback = procedure; cdecl;
  GXDrawDoneCallback = procedure; cdecl;
  GXDrawSyncCallback = procedure(token: cuint16); cdecl;
  GXTexRegionCallback = function(obj: PGXTexObj; mapid: cuint8): PGXTexRegion; cdecl;
  GXTlutRegionCallback = function(tlut_name: cuint32): PGXTlutRegion; cdecl;

var
  wgPipe : PWGPipe; cvar; external;

function GX_Init(base: pointer; size: cuint32): PGXFifoObj; cdecl; external;


procedure GX_InitFifoBase(fifo: PGXFifoObj; base: pointer; size: cuint32); cdecl; external;

procedure GX_InitFifoLimits(fifo: PGXFifoObj; hiwatermark, lowatermark: cuint32); cdecl; external;

procedure GX_InitFifoPtrs(fifo: PGXFifoObj; rd_ptr, wt_ptr: pointer); cdecl; external;

procedure GX_GetFifoPtrs(fifo: PGXFifoObj; rd_ptr, wt_ptr: pointer); cdecl; external;

procedure GX_SetCPUFifo(fifo: PGXFifoObj); cdecl; external;

procedure GX_SetGPFifo(fifo: PGXFifoObj); cdecl; external;

procedure GX_GetCPUFifo(fifo: PGXFifoObj); cdecl; external;

procedure GX_GetGPFifo(fifo: PGXFifoObj); cdecl; external;

function GX_GetFifoBase(fifo: PGXFifoObj): pointer; cdecl; external;

function GX_GetFifoCount(fifo: PGXFifoObj): cuint32; cdecl; external;

function GX_GetFifoSize(fifo: PGXFifoObj): cuint32; cdecl; external;

function GX_GetFifoWrap(fifo: PGXFifoObj): cuint8; cdecl; external;

function GX_SetDrawDoneCallback(cb: GXDrawDoneCallback): GXDrawDoneCallback; cdecl; external;

function GX_SetBreakPtCallback(cb: GXBreakPtCallback): GXBreakPtCallback; cdecl; external;

procedure GX_AbortFrame; cdecl; external;

procedure GX_Flush; cdecl; external;

procedure GX_SetMisc(token, value: cuint32); cdecl; external;

procedure GX_SetDrawDone; cdecl; external;

procedure GX_WaitDrawDone; cdecl; external;

function GX_GetDrawSync: cuint16; cdecl; external;

procedure GX_SetDrawSync(token: cuint16); cdecl; external;

function GX_SetDrawSyncCallback(cb: GXDrawSyncCallback): GXDrawSyncCallback; cdecl; external;

procedure GX_DisableBreakPt; cdecl; external;

procedure GX_EnableBreakPt(break_pt: pointer); cdecl; external;

procedure GX_DrawDone; cdecl; external;

procedure GX_TexModeSync; cdecl; external;

procedure GX_InvVtxCache; cdecl; external;

procedure GX_ClearVtxDesc; cdecl; external;

procedure GX_LoadProjectionMtx(mt: Mtx44; type_: cuint8); cdecl; external;

procedure GX_SetViewport(xOrig, yOrig, wd, ht, nearZ, farZ: f32); cdecl; external;

procedure GX_SetViewportJitter(xOrig, yOrig, wd, ht, nearZ, farZ: f32;
  field: cuint32); cdecl; external;

procedure GX_SetChanCtrl(channel: cint32;
  enable, ambsrc, matsrc, litmask, diff_fn, attn_fn: cuint8); cdecl; external;

procedure GX_SetChanAmbColor(channel: cint32; color: GXColor); cdecl; external;

procedure GX_SetChanMatColor(channel: cint32; color: GXColor); cdecl; external;

procedure GX_SetArray(attr: cuint32; ptr: pointer; stride: cuint8); cdecl; external;

procedure GX_SetVtxAttrFmt(vtxfmt: cuint8;
  vtxattr, comptype, compsize, frac: cuint32); cdecl; external;

procedure GX_SetVtxAttrFmtv(vtxfmt: cuint8; attr_list: PGXVtxAttrFmt); cdecl; external;

procedure GX_SetVtxDesc(attr, type_: cuint8); cdecl; external;

procedure GX_SetVtxDescv(attr_list: PGXVtxDesc); cdecl; external;

procedure GX_GetVtxDescv(attr_list: PGXVtxDesc); cdecl; external;

function GX_EndDispList: cuint32; cdecl; external;

procedure GX_Begin(primitve, vtxfmt: cuint8; vtxcnt: cuint16); cdecl; external;

procedure GX_BeginDispList(list: pointer; size: cuint32); cdecl; external;

procedure GX_CallDispList(list: pointer; nbytes: cuint32); cdecl; external;



procedure GX_End(); inline;

procedure GX_Position3f32(x, y, z: f32); inline;

procedure GX_Position3u16(x, y, z: cuint16); inline;

procedure GX_Position3s16(x, y, z: cint16); inline;

procedure GX_Position3u8(x, y, z: cuint8); inline;

procedure GX_Position3s8(x, y, z: cint8); inline;

procedure GX_Position2f32(x, y: f32); inline;

procedure GX_Position2u16(x, y: cuint16); inline;

procedure GX_Position2s16(x, y: cint16); inline;

procedure GX_Position2u8(x, y: cuint8); inline;

procedure GX_Position2s8(x, y: cint8); inline;

procedure GX_Position1x8(index: cuint8); inline;

procedure GX_Position1x16(index: cuint16); inline;

procedure GX_Normal3f32(nx, ny, nz: f32); inline;

procedure GX_Normal3s16(nx, ny, nz: cint16); inline;

procedure GX_Normal3s8(nx, ny, nz: cint8); inline;

procedure GX_Normal1x8(index: cuint8); inline;

procedure GX_Normal1x16(index: cuint16); inline;

procedure GX_Color4u8(r, g, b, a: cuint8); inline;

procedure GX_Color3u8(r, g, b: cuint8); inline;

procedure GX_Color3f32(r, g, b: f32); inline;

procedure GX_Color1u32(clr: cuint32); inline;

procedure GX_Color1u16(clr: cuint16); inline;

procedure GX_Color1x8(index: cuint8); inline;

procedure GX_Color1x16(index: cuint16); inline;

procedure GX_TexCoord2f32(s, t: f32); inline;

procedure GX_TexCoord2u16(s, t: cuint16); inline;

procedure GX_TexCoord2s16(s, t: cint16); inline;

procedure GX_TexCoord2u8(s, t: cuint8); inline;

procedure GX_TexCoord2s8(s, t: cint8); inline;

procedure GX_TexCoord1f32(s: f32); inline;

procedure GX_TexCoord1u16(s: cuint16); inline;

procedure GX_TexCoord1s16(s: cint16); inline;

procedure GX_TexCoord1u8(s: cuint8); inline;

procedure GX_TexCoord1s8(s: cint8); inline;

procedure GX_TexCoord1x8(index: cuint8); inline;

procedure GX_TexCoord1x16(index: cuint16); inline;

procedure GX_MatrixIndex1x8(index: cuint8); inline;

procedure GX_AdjustForOverscan(rmin, rmout: PGXRModeObj; hor, ver: cuint16); cdecl; external;

procedure GX_LoadPosMtxImm(mt: Mtx; pnidx: cuint32); cdecl; external;

procedure GX_LoadPosMtxIdx(mtxidx: cuint16; pnidx: cuint32); cdecl; external;


procedure GX_LoadNrmMtxImm(mt: Mtx; pnidx: cuint32); cdecl; external;


procedure GX_LoadNrmMtxIdx3x3(mtxidx: cuint16; pnidx: cuint32); cdecl; external;


procedure GX_LoadTexMtxImm(mt: Mtx; texidx: cuint32; type_: cuint8); cdecl; external;


procedure GX_LoadTexMtxIdx(mtxidx: cuint16; texidx: cuint32; type_: cuint8); cdecl; external;

procedure GX_SetCurrentMtx(mtx: cuint32); cdecl; external;

procedure GX_SetTevOp(tevstage, mode: cuint8); cdecl; external;

procedure GX_SetTevColor(tev_regid: cuint8; color: GXColor); cdecl; external;

procedure GX_SetTevColorS10(tev_regid: cuint8; color: GXColorS10); cdecl; external;

procedure GX_SetTevColorIn(tevstage, a, b, c, d: cuint8); cdecl; external;

procedure GX_SetTevAlphaIn(tevstage, a, b, c, d: cuint8); cdecl; external;

procedure GX_SetTevColorOp(
  tevstage, tevop, tevbias, tevscale, clamp, tevregid: cuint8); cdecl; external;

procedure GX_SetTevAlphaOp(
  tevstage, tevop, tevbias, tevscale, clamp, tevregid: cuint8); cdecl; external;

procedure GX_SetNumTexGens(nr: cuint32); cdecl; external;

procedure GX_SetTexCoordGen(texcoord: cuint16;
  tgen_typ, tgen_src, mtxsrc: cuint32); cdecl; external;

procedure GX_SetTexCoordGen2(texcoord: cuint16;
  tgen_typ, tgen_src, mtxsrc, normalize, postmtx: cuint32); cdecl; external;

procedure GX_SetZTexture(op, fmt: cuint8; bias: cuint32); cdecl; external;

procedure GX_SetZMode(enable, func, update_enable: cuint8); cdecl; external;

procedure GX_SetZCompLoc(before_tex: cuint8); cdecl; external;

procedure GX_SetLineWidth(width, fmt: cuint8); cdecl; external;

procedure GX_SetPointSize(width, fmt: cuint8); cdecl; external;

procedure GX_SetBlendMode(type_, src_fact, dst_fact, op: cuint8); cdecl; external;

procedure GX_SetCullMode(mode: cuint8); cdecl; external;

procedure GX_SetCoPlanar(enable: cuint8); cdecl; external;

procedure GX_EnableTexOffsets(coord, line_enable, point_enable: cuint8); cdecl; external;

procedure GX_SetClipMode(mode: cuint8); cdecl; external;

procedure GX_SetScissor(xOrigin, yOrigin, wd, ht: cuint32); cdecl; external;

procedure GX_SetScissorBoxOffset(xoffset, yoffset: cint32); cdecl; external;

procedure GX_SetNumChans(num: cuint8); cdecl; external;

procedure GX_SetTevOrder(tevstage, texcoord: cuint8; texmap: cuint32;
  color: cuint8); cdecl; external;

procedure GX_SetNumTevStages(num: cuint8); cdecl; external;

procedure GX_SetAlphaCompare(comp0, ref0, aop, comp1, ref1: cuint8); cdecl; external;

procedure GX_SetTevKColor(sel: cuint8; col: GXColor); cdecl; external;

procedure GX_SetTevKColorSel(tevstage, sel: cuint8); cdecl; external;

procedure GX_SetTevKAlphaSel(tevstage, sel: cuint8); cdecl; external;

procedure GX_SetTevKColorS10(sel: cuint8; col: GXColorS10); cdecl; external;

procedure GX_SetTevSwapMode(tevstage, ras_sel, tex_sel: cuint8); cdecl; external;

procedure GX_SetTevSwapModeTable(swapid, r, g, b, a: cuint8); cdecl; external;

procedure GX_SetTevIndirect(
  tevstage, indtexid, format, bias, mtxid, wrap_s, wrap_t, addprev, utclod, a: cuint8); cdecl; external;

procedure GX_SetTevDirect(tevstage: cuint8); cdecl; external;

procedure GX_SetNumIndStages(nstages: cuint8); cdecl; external;

procedure GX_SetIndTexOrder(indtexstage, texcoord, texmap: cuint8); cdecl; external;

procedure GX_SetIndTexCoordScale(indtexid, scale_s, scale_t: cuint8); cdecl; external;

procedure GX_SetFog(type_: cuint8; startz, endz, nearz, farz: f32; col: GXColor); cdecl; external;

procedure GX_SetFogRangeAdj(enable: cuint8; center: cuint16;
  table: PGXFogAdjTbl); cdecl; external;


procedure GX_SetFogColor(color: GXColor); cdecl; external;


procedure GX_InitFogAdjTable(table: PGXFogAdjTbl; width: cuint16; projmtx: f32); cdecl; external;

procedure GX_SetIndTexMatrix(indtexmtx: cuint8; offset_mtx: f32;
  scale_exp: cint8); cdecl; external;

procedure GX_SetTevIndBumpST(tevstage, indstage, mtx_sel: cuint8); cdecl; external;

procedure GX_SetTevIndBumpXYZ(tevstage, indstage, mtx_sel: cuint8); cdecl; external;

procedure GX_SetTevIndTile(tevstage, indtexid: cuint8;
  tilesize_x, tilesize_y, tilespacing_x, tilespacing_y: cuint16;
  indtexfmt, indtexmtx, bias_sel, alpha_sel: cuint8); cdecl; external;


(*!
 * \fn void GX_SetTevIndRepeat(u8 tevstage)
 * \brief Set a given TEV stage to use the same texture coordinates as were computed in the previous stage.
 *
 * \note This is only useful when the previous stage texture coordinates took more than one stage to compute, as is the case for GX_SetTevIndBumpST().
 *
 * \param[in] tevstage \ref tevstage to modify
 *
 * \return none
  *)

procedure GX_SetTevIndRepeat(tevstage: cuint8); cdecl; external;

(*!
 * \fn void GX_SetColorUpdate(u8 enable)
 * \brief Enables or disables color-buffer updates when rendering into the Embedded Frame Buffer (EFB).
 *
 * \note This function also affects whether the color buffer is cleared during copies; see GX_CopyDisp() and GX_CopyTex().
 *
 * \param[in] enable enables color-buffer updates with <tt>GX_TRUE</tt>
 *
 * \return none
  *)

procedure GX_SetColorUpdate(enable: cuint8); cdecl; external;


(*!
 * \fn void GX_SetAlphaUpdate(u8 enable)
 * \brief Enables or disables alpha-buffer updates of the Embedded Frame Buffer (EFB).
 *
 * \note This function also affects whether the alpha buffer is cleared during copy operations; see GX_CopyDisp() and GX_CopyTex().<br><br>
 *
 * \note The only EFB pixel format supporting an alpha buffer is <tt>GX_PF_RGBA6_Z24</tt>; see GX_SetPixelFmt(). The alpha \a enable is ignored for non-alpha
 * pixel formats.
 *
 * \param[in] enable enables alpha-buffer updates with <tt>GX_TRUE</tt>
 *
 * \return none
  *)

procedure GX_SetAlphaUpdate(enable: cuint8); cdecl; external;


(*!
 * \fn void GX_SetPixelFmt(u8 pix_fmt,u8 z_fmt)
 * \brief Sets the format of pixels in the Embedded Frame Buffer (EFB).
 *
 * \details There are two non-antialiased \a pix_fmts: <tt>GX_PF_RGB8_Z24</tt> and <tt>GX_PF_RGBA6_Z24</tt>. The stride of the EFB is fixed at 640 pixels. The
 * non-antialiased EFB has 528 lines available.
 *
 * When \a pix_fmt is set to <tt>GX_PF_RGB565_Z16</tt>, multi-sample antialiasing is enabled. In order to get proper results, one must also call GX_SetCopyFilter().
 * The position of the subsamples and the antialiasing filter coefficients are set using GX_SetCopyFilter(). When antialiasing, three 16b color/Z
 * samples are computed for each pixel, and the total available number of pixels in the EFB is reduced by half (640 pixels x 264 lines). This function also sets the
 * compression type for 16-bit Z formats, which allows trading off Z precision for range. The following guidelines apply:<br><br>
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;a) far/near ratio <= 2^16, use <tt>GX_ZC_LINEAR</tt><br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;b) far/near ratio <= 2^18, use <tt>GX_ZC_NEAR</tt><br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;c) far/near ratio <= 2^20, use <tt>GX_ZC_MID</tt><br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;d) far/near ratio <= 2^24, use <tt>GX_ZC_FAR</tt><br><br>
 *
 * It is always best to use as little compression as possible (choice "a" is least compressed, choice "d" is most compressed). You get less precision with higher compression.
 * The "far" in the above list does not necessarily refer to the far clipping plane. You should think of it as the farthest object you want correct occlusion for.
 *
 * \note This function also controls antialiasing (AA) mode.<br><br>
 *
 * \note Since changing pixel format requires the pixel pipeline to be synchronized, the use of this function causes stall of the graphics processor as a result. Therefore,
 * you should avoid redundant calls of this function.
 *
 * \param[in] pix_fmt <tt>GX_PF_RGB8_Z24</tt> or <tt>GX_PF_RGBA6_Z24</tt> for non-AA, <tt>GX_PF_RGB565_Z16</tt> for AA
 * \param[in] z_fmt \ref zfmt to use
 *
 * \return none
  *)

procedure GX_SetPixelFmt(pix_fmt, z_fmt: cuint8); cdecl; external;


(*!
 * \fn void GX_SetDither(u8 dither)
 * \brief Enables or disables dithering.
 *
 * \details A 4x4 Bayer matrix is used for dithering.
 *
 * \note Only valid when the pixel format (see GX_SetPixelFmt()) is either <tt>GX_PF_RGBA6_Z24</tt> or <tt>GX_PF_RGB565_Z16</tt>.<br><br>
 *
 * \note Dithering should probably be turned off if you are planning on using the result of rendering for comparisons (e.g. outline rendering
 * algorithm that writes IDs to the alpha channel, copies the alpha channel to a texture, and later compares the texture in the TEV).
 *
 * \param[in] dither enables dithering if <tt>GX_TRUE</tt> is given and pixel format is one of the two above, otherwise disabled
 *
 * \return none
  *)

procedure GX_SetDither(dither: cuint8); cdecl; external;


(*!
 * \fn void GX_SetDstAlpha(u8 enable,u8 a)
 * \brief Sets a constant alpha value for writing to the Embedded Frame Buffer (EFB).
 *
 * \note To be effective, the EFB pixel type must have an alpha channel (see GX_SetPixelFmt()). The alpha compare operation (see
 * GX_SetAlphaCompare()) and blending operations (see GX_SetBlendMode()) still use source alpha (output from the last TEV stage) but when
 * writing the pixel color, the constant alpha will replace the pixel alpha in the EFB.
 *
 * \param[in] enable \a a will be written to the framebuffer if <tt>GX_ENABLE</tt> is here and frame buffer pixel format supports destination alpha
 * \param[in] a constant alpha value
 *
 * \return none
  *)

procedure GX_SetDstAlpha(enable, a: cuint8); cdecl; external;


(*!
 * \fn void GX_SetFieldMask(u8 even_mask,u8 odd_mask)
 * \brief selectively enables and disables interlacing of the frame buffer image.
 *
 * \details This function is used when rendering fields to an interlaced Embedded Frame Buffer (EFB).
 *
 * \note When the mask is <tt>GX_FALSE</tt>, that field will not be written to the EFB, but the other field will be computed. In other words, you pay the
 * fill rate price of a frame to produce a field.
 *
 * \param[in] even_mask whether to write pixels with even Y coordinate
 * \param[in] odd_mask whether to write pixels with odd Y coordinate
 *
 * \return none
  *)

procedure GX_SetFieldMask(even_mask, odd_mask: cuint8); cdecl; external;


(*!
 * \fn void GX_SetFieldMode(u8 field_mode,u8 half_aspect_ratio)
 * \brief Controls various rasterization and texturing parameters that relate to field-mode and double-strike rendering.
 *
 * \details In field-mode rendering, one must adjust the vertical part of the texture LOD computation to account for the fact that pixels cover only half of
 * the space from one rendered scan line to the next (with the other half of the space filled by a pixel from the other field).  In both field-mode and
 * double-strike rendering, one must adjust the aspect ratio for points and lines to account for the fact that pixels will be double-height when displayed
 * (the pixel aspect ratio is 1/2).
 *
 * \note The values set here usually come directly from the render mode. The \a field_rendering flags goes straight into \a field_mode. The \a half_aspect_ratio
 * parameter is true if the \a xfbHeight is half of the \a viHeight, false otherwise.<br><br>
 *
 * \note GX_Init() sets both fields according to the default render mode.<br><br>
 *
 * \note On production hardware (i.e. a retail GameCube), only line aspect-ratio adjustment is implemented. Points are not adjusted.
 *
 * \param[in] field_mode adjusts texture LOD computation as described above if true, otherwise does not
 * \param[in] half_aspect_ratio adjusts line aspect ratio accordingly, otherwise does not
 *
 * \return none
  *)

procedure GX_SetFieldMode(field_mode, half_aspect_ratio: cuint8); cdecl; external;


(*!
 * \fn f32 GX_GetYScaleFactor(u16 efbHeight,u16 xfbHeight)
 * \brief Calculates an appropriate Y scale factor value for GX_SetDispCopyYScale() based on the height of the EFB and
 *        the height of the XFB.
 *
 * \param[in] efbHeight Height of embedded framebuffer. Range from 2 to 528. Should be a multiple of 2.
 * \param[in] xfbHeight Height of external framebuffer. Range from 2 to 1024. Should be equal or greater than \a efbHeight.
 *
 * \return Y scale factor which can be used as argument of GX_SetDispCopyYScale().
  *)

function GX_GetYScaleFactor(efbHeight, xfbHeight: cuint16): f32; cdecl; external;


(*!
 * \fn u32 GX_SetDispCopyYScale(f32 yscale)
 * \brief Sets the vertical scale factor for the EFB to XFB copy operation.
 *
 * \details The number of actual lines copied is returned, based on the current EFB height. You can use this number to allocate the proper XFB size. You
 * have to call GX_SetDispCopySrc() prior to this function call if you want to get the number of lines by using this function.
 *
 * \param[in] yscale Vertical scale value. Range from 1.0 to 256.0.
 *
 * \return Number of lines that will be copied.
  *)

function GX_SetDispCopyYScale(yscale: f32): cuint32; cdecl; external;


(*!
 * \fn void GX_SetDispCopySrc(u16 left,u16 top,u16 wd,u16 ht)
 * \brief Sets the source parameters for the EFB to XFB copy operation.
 *
 * \param[in] left left most source pixel to copy. Must be a multiple of 2 pixels.
 * \param[in] top top most source line to copy. Must be a multiple of 2 lines.
 * \param[in] wd width in pixels to copy. Must be a multiple of 2 pixels.
 * \param[in] ht height in lines to copy. Must be a multiple of 2 lines.
 *
 * \return none
  *)

procedure GX_SetDispCopySrc(left, top, wd, ht: cuint16); cdecl; external;


(*!
 * \fn void GX_SetDispCopyDst(u16 wd,u16 ht)
 * \brief Sets the witdth and height of the display buffer in pixels.
 *
 * \details The application typical renders an image into the EFB(source) and then copies it into the XFB(destination) in main memory. \a wd
 * specifies the number of pixels between adjacent lines in the destination buffer and can be different than the width of the EFB.
 *
 * \param[in] wd Distance between successive lines in the XFB, in pixels. Must be a multiple of 16.
 * \param[in] ht Height of the XFB in lines.
 *
 * \return none
  *)

procedure GX_SetDispCopyDst(wd, ht: cuint16); cdecl; external;


(*!
 * \fn void GX_SetCopyClamp(u8 clamp)
 * \brief Sets the vertical clamping mode to use during the EFB to XFB or texture copy.
 *
 * \param[in] clamp bit-wise OR of desired \ref xfbclamp. Use <tt>GX_CLAMP_NONE</tt> for no clamping.
 *
 * \return none
  *)

procedure GX_SetCopyClamp(clamp: cuint8); cdecl; external;


(*!
 * \fn void GX_SetDispCopyGamma(u8 gamma)
 * \brief Sets the gamma correction applied to pixels during EFB to XFB copy operation.
 *
 * \param[in] gamma \ref gammamode
 *
 * \return none
  *)

procedure GX_SetDispCopyGamma(gamma: cuint8); cdecl; external;


(*!
 * \fn void GX_SetCopyFilter(u8 aa,u8 sample_pattern[12][2],u8 vf,u8 vfilter[7])
 * \brief Sets the subpixel sample patterns and vertical filter coefficients used to filter subpixels into pixels.
 *
 * \details This function normally uses the \a aa, \a sample_pattern and \a vfilter provided by the render mode struct:<br><br>
 *
 * \code GXRModeObj* rmode = VIDEO_GetPreferredMode(NULL);
 * GX_SetCopyFilter(rmode->aa,rmode->sample_pattern,GX_TRUE,rmode->vfilter); \endcode
 *
 * \note In order to make use of the \a sample_pattern, antialiasing must be enabled by setting the Embedded Frame Buffer (EFB) format to
 * <tt>GX_PF_RGB565_Z16</tt>; see GX_SetPixelFmt().
 *
 * \param[in] aa utilizes \a sample_pattern if <tt>GX_TRUE</tt>, otherwise all sample points are centered
 * \param[in] sample_pattern array of coordinates for sample points; valid range is 1 - 11 inclusive
 * \param[in] vf use \a vfilter if <tt>GX_TRUE</tt>, otherwise use default 1-line filter
 * \param[in] vfilter vertical filter coefficients; valid coefficient range is 0 - 63 inclusive; sum should equal 64
 *
 * \return none
  *)

type
  TSamplePattern = array [0..11, 0..1] of cuint8;
  TVFilter = array [0..6] of cuint8;
procedure GX_SetCopyFilter(aa: cuint8; sample_pattern: TSamplePattern; vf: cuint8; vfilter: TVFilter); cdecl; external;


(*!
 * \fn void GX_SetDispCopyFrame2Field(u8 mode)
 * \brief Determines which lines are read from the Embedded Frame Buffer (EFB) when using GX_CopyDisp().
 *
 * \details Specifically, it determines whether all lines, only even lines, or only odd lines are read.
 *
 * \note The opposite function, which determines whether all lines, only even lines or only odd lines are <i>written</i> to the EFB, is GX_SetFieldMask().<br><br>
 *
 * \note Only applies to display copies, GX_CopyTex() always uses the <tt>GX_COPY_PROGRESSIVE</tt> mode.
 *
 * \param[in] mode \ref copymode to determine which field to copy (or both)
 *
 * \return none
  *)

procedure GX_SetDispCopyFrame2Field(mode: cuint8); cdecl; external;


(*!
 * \fn void GX_SetCopyClear(GXColor color,u32 zvalue)
 * \brief Sets color and Z value to clear the EFB to during copy operations.
 *
 * \details These values are used during both display copies and texture copies.
 *
 * \param[in] color RGBA color (8-bit/component) to use during clear operation.
 * \param[in] zvalue 24-bit Z value to use during clear operation. Use the constant <tt>GX_MAX_Z24</tt> to specify the maximum depth value.
 *
 * \return none
  *)

procedure GX_SetCopyClear(color: GXColor; zvalue: cuint32); cdecl; external;


(*!
 * \fn void GX_CopyDisp(void *dest,u8 clear)
 * \brief Copies the embedded framebuffer (EFB) to the external framebuffer(XFB) in main memory.
 *
 * \note The stride of the XFB is set using GX_SetDispCopyDst(). The source image in the EFB is described using GX_SetDispCopySrc().<br><br>
 *
 * \note The graphics processor will stall all graphics commands util the copy is complete.<br><br>
 *
 * \note If the \a clear flag is true, the color and Z buffers will be cleared during the copy. They will be cleared to the constant
 * values set using GX_SetCopyClear().
 *
 * \param[in] dest pointer to the external framebuffer. \a dest should be 32B aligned.
 * \param[in] clear flag that indicates framebuffer should be cleared if <tt>GX_TRUE</tt>.
 *
 * \return none
  *)

procedure GX_CopyDisp(dest: pointer; clear: cuint8); cdecl; external;


(*!
 * \fn void GX_SetTexCopySrc(u16 left,u16 top,u16 wd,u16 ht)
 * \brief Sets the source parameters for the Embedded Frame Buffer (EFB) to texture image copy.
 *
 * \param[in] left left-most source pixel to copy, multiple of two
 * \param[in] top top-most source line to copy, multiple of two
 * \param[in] wd width to copy in pixels, multiple of two
 * \param[in] ht height to copy in pixels, multiple of two
 *
 * \return none
  *)

procedure GX_SetTexCopySrc(left, top, wd, ht: cuint16); cdecl; external;


(*!
 * \fn void GX_SetTexCopyDst(u16 wd,u16 ht,u32 fmt,u8 mipmap)
 * \brief This function sets the width and height of the destination texture buffer in texels.
 *
 * \details This function is useful when creating textures using the Graphics Processor (GP). If the \a clear flag is set to <tt>GX_TRUE</tt>, the
 * EFB will be cleared to the current color (see GX_SetCopyClear()) during the copy operation.
 *
 * \param[in] wd pointer to the image buffer in main memory. \a dest should be 32B aligned.
 * \param[in] ht flag that indicates framebuffer should be cleared if <tt>GX_TRUE</tt>.
 * \param[in] fmt \ref texfmt
 * \param[in] mipmap
 *
 * \return none
  *)

procedure GX_SetTexCopyDst(wd, ht: cuint16; fmt: cuint32; mipmap: cuint8); cdecl; external;


(*!
 * \fn void GX_CopyTex(void *dest,u8 clear)
 * \brief Copies the embedded framebuffer (EFB) to the texture image buffer \a dest in main memory.
 *
 * \details This is useful when creating textures using the Graphics Processor (GP). If the \a clear flag is set to <tt>GX_TRUE</tt>, the EFB will be cleared
 * to the current color(see GX_SetCopyClear()) during the copy operation.
 *
 * \param[in] dest pointer to the image buffer in main memory. \a dest should be 32B aligned.
 * \param[in] clear flag that indicates framebuffer should be cleared if <tt>GX_TRUE</tt>.
 *
 * \return none
  *)

procedure GX_CopyTex(dest: pointer; clear: cuint8); cdecl; external;


(*!
 * \fn void GX_PixModeSync()
 * \brief Causes the GPU to wait for the pipe to flush.
 *
 * \details This function inserts a synchronization command into the graphics FIFO. When the GPU sees this command it will allow the rest of the pipe to
 * flush before continuing. This command is useful in certain situation such as after using GX_CopyTex() and before a primitive that uses the copied texture.
 *
 * \note The command is actually implemented by writing the control register that determines the format of the embedded frame buffer (EFB). As a result, care
 * should be used if this command is placed within a display list.
 *
 * \return none
  *)

procedure GX_PixModeSync; cdecl; external;


(*!
 * \fn void GX_ClearBoundingBox()
 * \brief Clears the bounding box values before a new image is drawn.
 *
 * \details The graphics hardware keeps track of a bounding box of pixel coordinates that are drawn in the Embedded Frame Buffer (EFB).
 *
 * \return none
  *)

procedure GX_ClearBoundingBox; cdecl; external;


(*!
 * \fn GX_PokeAlphaMode(u8 func,u8 threshold)
 * \brief Sets a threshold which is compared to the alpha of pixels written to the Embedded Frame Buffer (EFB) using the GX_Poke*() functions.
 *
 * \details The compare function order is:<br><br>
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;src_alpha \a func \a threshold
 *
 * \note The alpha compare function can be used to conditionally write pixels to the EFB using the source alpha channel as a template. If the compare function is
 * true, the source color will be written to the EFB based on the result of the Z compare (see GX_PokeZMode()). If the alpha compare function is false, the source
 * color is not written to the EFB.<br><br>
 *
 * \note The alpha compare test happens before the Z compare and before blending (see GX_PokeBlendMode()).
 *
 * \param[in] func \ref compare to use
 * \param[in] threshold to which the source alpha will be compared to
 *
 * \return none
  *)

procedure GX_PokeAlphaMode(func, threshold: cuint8); cdecl; external;


(*!
 * \fn void GX_PokeAlphaUpdate(u8 update_enable)
 * \brief Enables or disables alpha-buffer updates for GX_Poke*() functions.
 *
 * \details The normal rendering state (set by GX_SetAlphaUpdate()) is not affected.
 *
 * \param[in] update_enable enables alpha-buffer updates with <tt>GX_TRUE</tt>, otherwise does not
 *
 * \return none
  *)

procedure GX_PokeAlphaUpdate(update_enable: cuint8); cdecl; external;


(*!
 * \fn void GX_PokeColorUpdate(u8 update_enable)
 * \brief Enables or disables color-buffer updates when writing the Embedded Frame Buffer (EFB) using the GX_Poke*() functions.
 *
 * \param[in] update_enable enables color-buffer updates with <tt>GX_TRUE</tt>, otherwise does not
 *
 * \return none
  *)

procedure GX_PokeColorUpdate(update_enable: cuint8); cdecl; external;


(*!
 * \fn void GX_PokeDither(u8 dither)
 * \brief Enables dithering when writing the Embedded Frame Buffer (EFB) using GX_Poke*() functions.
 *
 * \note The \a dither enable is only valid when the pixel format (see GX_SetPixelFmt()) is either <tt>GX_PF_RGBA6_Z24</tt> or <tt>GX_PF_RGB565_Z16</tt>.<br><br>
 *
 * \note A 4x4 Bayer matrix is used for dithering.
 *
 * \param[in] if set to <tt>GX_TRUE</tt> and pixel format is one of the above, dithering is enabled; otherwise disabled
 *
 * \return none
  *)

procedure GX_PokeDither(dither: cuint8); cdecl; external;


(*!
 * \fn void GX_PokeBlendMode(u8 type,u8 src_fact,u8 dst_fact,u8 op)
 * \brief Determines how the source image, is blended with the current Embedded Frame Buffer (EFB).
 *
 * \defails When type is set to <tt>GX_BM_NONE</tt>, no color data is written to the EFB. When type is set to <tt>GX_BM_BLEND</tt>, the source and EFB pixels
 * are blended using the following equation:<br><br>
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<i>dst_pix_clr</i> = <i>src_pix_clr</i> * \a src_fact + <i>dst_pix_clr</i> * \a dst_fact<br><br>
 *
 * When type is set to <tt>GX_BM_SUBTRACT</tt>, the destination pixel is computed as follows:<br><br>
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<i>dst_pix_clr</i> = <i>dst_pix_clr</i> - <i>src_pix_clr [clamped to zero]</i><br><br>
 *
 * Note that \a src_fact and \a dst_fact are not part of the equation.
 *
 * \note \a dst_fact can be used only when the frame buffer has <tt>GX_PF_RGBA6_Z24</tt> as the pixel format (see GX_SetPixelFmt()).<br><br>
 *
 * \note When type is set to <tt>GX_BM_LOGIC</tt>, the source and EFB pixels are blended using logical bitwise operations.<br><br>
 *
 * \note This function does not effect the normal rendering state; see GX_SetBlendMode().
 *
 * \param[in] type \ref blendmode
 * \param[in] src_fact source \ref blendfactor; the pixel color produced by the graphics processor is multiplied by this factor
 * \param[in] dst_fact destination \ref blendfactor; the current frame buffer pixel color is multiplied by this factor
 * \param[in] op \ref logicop to use
  *)

procedure GX_PokeBlendMode(type_, src_fact, dst_fact, op: cuint8); cdecl; external;


(*!
 * \fn void GX_PokeAlphaRead(u8 mode)
 * \brief Determines what value of alpha will be read from the Embedded Frame Buffer (EFB).
 *
 * \details The mode only applies to GX_Peek*() functions.
 *
 * \note This feature works no matter what pixel type (see GX_SetPixelFmt()) you are using. If you are using the EFB with alpha plane, it is
 * recommended that you use <tt>GX_READ_NONE</tt> so that you can read correct alpha value from the EFB. If you are using the EFB with no alpha, you should
 * set either of <tt>GX_READ_00</tt> or <tt>GX_READ_FF</tt> in order to get a certain value.<br><br>
 *
 * \param[in] mode \ref alphareadmode that determines value of alpha read from a frame buffer with no alpha channel.
 *
 * \return none
  *)

procedure GX_PokeAlphaRead(mode: cuint8); cdecl; external;


(*!
 * \fn void GX_PokeDstAlpha(u8 enable,u8 a)
 * \brief Sets a constant alpha value for writing to the Embedded Frame Buffer (EFB).
 *
 * \details The EFB pixel type must have an alpha channel for this function to be effective (see GX_SetPixelFmt()). The blending operations (see
 * GX_PokeBlendMode()) still use source alpha but when writing the pixel color, the constant \a a will replace the pixel alpha in the EFB.
 *
 * \param[in] enable if set to <tt>GX_ENABLE</tt> and pixel format supports dest alpha, \a a will be written to the framebuffer
 * \param[in] a constant alpha value
 *
 * \return none
  *)

procedure GX_PokeDstAlpha(enable, a: cuint8); cdecl; external;


(*!
 * \fn void GX_PokeARGB(u16 x,u16 y,GXColor color)
 * \brief Allows the CPU to write \a color directly to the Embedded Frame Buffer (EFB) at position \a x,\a y.
 *
 * \details The alpha value in \a color can be compared with the current alpha threshold (see GX_PokeAlphaMode()). The color will be blended
 * into the EFB using the blend mode set by GX_PokeBlendMode().
 *
 * \note For an antialiased frame buffer, all 3 subsamples of a pixel are affected by the poke.
 *
 * \param[in] x coordinate, in pixels; must be 0 - 639 inclusive
 * \param[in] y coordinate, in lines; must be 0 - 527 inclusive
 * \param[in] color color to write at the location
 *
 * \return none
  *)

procedure GX_PokeARGB(x, y: cuint16; color: GXColor); cdecl; external;


(*!
 * \fn void GX_PeekARGB(u16 x,u16 y,GXColor *color)
 * \brief Allows the CPU to read a color value directly from the Embedded Frame Buffer (EFB) at position \a x,\a y.
 *
 * \note For an antialiased frame buffer, only subsample 0 of a pixel is read.
 *
 * \param[in] x coordinate, in pixels; must be 0 - 639 inclusive
 * \param[in] y coordinate, in lines; must be 0 - 527 inclusive
 * \param[out] color struct to store color in
 *
 * \return none
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure GX_PeekARGB(x, y: cuint16; var color: GXColor); cdecl; external;
=======
procedure GX_PeekARGB(x, y: cuint16; color: PGXColor); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_PeekARGB(x, y: cuint16; color: PGXColor); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_PeekARGB(x, y: cuint16; color: PGXColor); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_PeekARGB(x, y: cuint16; color: PGXColor); cdecl; external;
>>>>>>> origin/cpstrnew


(*!
 * \fn void GX_PokeZ(u16 x,u16 y,u32 z)
 * \brief Allows the CPU to write a z value directly to the Embedded Frame Buffer (EFB) at position \a x,\a y.
 *
 * \details The \a z value can be compared with the current contents of the EFB. The Z compare fuction is set using GX_PokeZMode().
 *
 * \note The \a z value should be in the range of 0x00000000 <= \a z < 0x00FFFFFF in the case of non-antialiased frame buffer. For an antialiased
 * frame buffer, the \a z value should be in the compressed 16-bit format (0x00000000 <= \a z <= 0x0000FFFF), and the poke will affect all 3
 * subsamples of a pixel.
 *
 * \param[in] x coordinate, in pixels; must be 0 - 639 inclusive
 * \param[in] y coordinate, in lines; must be 0 - 527 inclusive
 * \param[in] z value to write at position \a x,\a y in the EFB
 *
 * \return none
  *)

procedure GX_PokeZ(x, y: cuint16; z: cuint32); cdecl; external;


(*!
 * \fn void GX_PeekZ(u16 x,u16 y,u32 *z)
 * \brief Allows the CPU to read a z value directly from the Embedded Frame Buffer (EFB) at position x,y.
 *
 * \details The z value is raw integer value from the Z buffer.
 *
 * \note The value range is 24-bit when reading from non-antialiased frame buffer. When reading from an antialiased frame buffer, subsample
 * 0 is read and returned. The value will be compressed 16-bit form in this case.
 *
 * \param[in] x coordinate, in pixels; must be 0 - 639 inclusive
 * \param[in] y coordinate, in lines; must be 0 - 527 inclusive
 * \param[out] z pointer to a returned Z value
 *
 * \return none
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure GX_PeekZ(x, y: cuint16; var z: cuint32); cdecl; external;
=======
procedure GX_PeekZ(x, y: cuint16; z: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_PeekZ(x, y: cuint16; z: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_PeekZ(x, y: cuint16; z: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_PeekZ(x, y: cuint16; z: pcuint32); cdecl; external;
>>>>>>> origin/cpstrnew


(*!
 * \fn void GX_PokeZMode(u8 comp_enable,u8 func,u8 update_enable)
 * \brief Sets the Z-buffer compare mode when writing the Embedded Frame Buffer (EFB).
 *
 * \details The result of the Z compare is used to conditionally write color values to the EFB. The Z value will be updated according to the
 * result of the compare if Z update is enabled.
 *
 * When \a comp_enable is set to <tt>GX_DISABLE</tt>, poke Z buffering is disabled and the Z buffer is not updated. The \a func parameter determines the
 * comparison that is performed. In the comparison function, the poked Z value is on the left while the Z value from the Z buffer is on the
 * right. If the result of the comparison is false, the poked Z value is discarded. The parameter \a update_enable determines whether or not the
 * Z buffer is updated with the new Z value after a comparison is performed.
 *
 * \note The normal rendering Z mode (set by GX_SetZMode()) is not affected by this function.<br><br>
 *
 * \note Even if update_enable is <tt>GX_FALSE</tt>, compares may still be enabled.
 *
 * \param[in] comp_enable enables comparisons with source and destination Z values if <tt>GX_TRUE</tt>
 * \param[in] func \ref compare function to use
 * \param[in] update_enable enables Z-buffer updates when <tt>GX_TRUE</tt>
 *
 * \return none
  *)

procedure GX_PokeZMode(comp_enable, func, update_enable: cuint8); cdecl; external;


(*!
 * \fn u32 GX_GetTexObjFmt(GXTexObj *obj)
 * \brief Returns the texture format described by texture object \a obj.
 *
 * \note Use GX_InitTexObj() or GX_InitTexObjCI() to initialize the texture format.
 *
 * \param[in] obj ptr to a texture object
 *
 * \return texture format of the given texture object
  *)

function GX_GetTexObjFmt(obj: PGXTexObj): cuint32; cdecl; external;


(*!
 * \fn u32 GX_GetTexObjMipMap(GXTexObj *obj)
 * \brief Returns the texture mipmap enable described by texture object \a obj.
 *
 * \note Use GX_InitTexObj() or GX_InitTexObjCI() to initialize the texture mipmap enable.
 *
 * \param[in] obj ptr to a texture object
 *
 * \return mipmap enable flag
  *)

function GX_GetTexObjMipMap(obj: PGXTexObj): cuint32; cdecl; external;


function GX_GetTexObjUserData(obj: PGXTexObj): pointer; cdecl; external;


(*!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * \fn void* GX_GetTexObjData(GXTexObj *obj)
 * \brief Used to get a pointer to texture data from the \ref GXTexObj structure.
 *
 * \note The returned pointer is a physical address.
 *
 * \param[in] obj ptr to a texture object
 *
 * \return Physical pointer to texture data.
 *)
function GX_GetTexObjData(obj: PGXTexObj): pointer; cdecl; external;

(*!
 * \fn u8 GX_GetTexObjWrapS(GXTexObj* obj)
 * \brief Returns the texture wrap s mode described by texture object \a obj.
 *
 * \note Use GX_InitTexObj() or GX_InitTexObjCI() to initialize the texture wrap s mode.
 *
 * \param[in] obj ptr to a texture object
 *
 * \return wrap s mode
 *)
function GX_GetTexObjWrapS(obj: PGXTexObj): cuint8; cdecl; external;

(*!
 * \fn u8 GX_GetTexObjWrapT(GXTexObj* obj)
 * \brief Returns the texture wrap t mode described by texture object \a obj.
 *
 * \note Use GX_InitTexObj() or GX_InitTexObjCI() to initialize the texture wrap t mode.
 *
 * \param[in] obj ptr to a texture object
 *
 * \return wrap t mode
 *)
function GX_GetTexObjWrapT(obj: PGXTexObj): cuint8; cdecl; external;

(*!
 * \fn u16 GX_GetTexObjHeight(GXTexObj* obj)
 * \brief Returns the texture height described by texture object \a obj.
 *
 * \note Use GX_InitTexObj() or GX_InitTexObjCI() to initialize the texture height.
 *
 * \param[in] obj ptr to a texture object
 *
 * \return texture height
 *)
function GX_GetTexObjHeight(obj: PGXTexObj): cuint16; cdecl; external;

(*!
 * \fn u16 GX_GetTexObjWidth(GXTexObj* obj)
 * \brief Returns the texture width described by texture object \a obj.
 *
 * \note Use GX_InitTexObj() or GX_InitTexObjCI() to initialize the texture width.
 *
 * \param[in] obj ptr to a texture object
 *
 * \return texture width
 *)
function GX_GetTexObjWidth(obj: PGXTexObj): cuint16; cdecl; external;

(*!
 * \fn void GX_GetTexObjAll(GXTexObj* obj, void** image_ptr, u16* width, u16* height, u8* format, u8* wrap_s, u8* wrap_t, u8* mipmap);
 * \brief Returns the parameters described by a texture object. Texture objects are used to describe all the parameters associated with a texture, including size, format, wrap modes, filter modes, etc. Texture objects are initialized using either GX_InitTexObj() or, for color index format textures, GX_InitTexObjCI().
 *
 * \param[in] obj ptr to a texture object
 * \param[out] image_ptr Returns a physical pointer to the image data for a texture.
 * \param[out] width Returns the width of the texture or LOD 0 for mipmaps
 * \param[out] height Returns the height of the texture or LOD 0 for mipmaps
 * \param[out] format Returns the texel format
 * \param[out] mipmap Returns the mipmap enable flag.
 *
 * \return none
 *)
procedure GX_GetTexObjAll(obj: PGXTexObj; var image_ptr: Pointer; var width: cuint16; var height: cuint16; var format: cuint8; var wrap_s: cuint8; var wrap_t: cuint8; var mipmap: cuint8); cdecl; external;


(*!
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 * \fn u32 GX_GetTexBufferSize(u16 wd,u16 ht,u32 fmt,u8 mipmap,u8 maxlod)
 * \brief Returns the amount of memory in bytes needed to store a texture of the given size and \a fmt.
 *
 * \details If the \a mipmap flag is <tt>GX_TRUE</tt>, then the size of buffer needed for the mipmap pyramid up to \a maxlod will be returned.
 * \a maxlod will be clamped to the number of LODs possible given the map \a wd and \a ht. For mipmaps, \a wd and \a ht must be a power of two.
 *
 * \note This function takes into account the tiling and padding requirements of the GameCube's native texture format. The resulting size can be used
 * along with memalign() to allocate texture buffers (see GX_CopyTex()).
 *
 * \param[in] wd width of the texture in texels
 * \param[in] ht height of the texture in texels
 * \param[in] fmt format of the texture; use GX_TexFmt() or GX_CITexFmt() to get it
 * \param[in] mipmap flag indicating whether or not the texture is a mipmap
 *
 * \return number of bytes needed for the texture, including tile padding
  *)

function GX_GetTexBufferSize(wd, ht: cuint16; fmt: cuint32;
  mipmap, maxlod: cuint8): cuint32; cdecl; external;


(*!
 * \fn void GX_InvalidateTexAll()
 * \brief Invalidates the current caches of the Texture Memory (TMEM).
 *
 * \details It takes about 512 GP clocks to invalidate all the texture caches.
 *
 * \note Preloaded textures (see GX_PreloadEntireTexture()) are not affected.
 *
 * \return none
  *)

procedure GX_InvalidateTexAll; cdecl; external;


(*!
 * \fn void GX_InvalidateTexRegion(GXTexRegion *region)
 * \brief Invalidates the texture cache in Texture Memory (TMEM) described by \a region.
 *
 * \details This function should be called when the CPU is used to modify a texture in main memory, or a new texture is loaded into main memory that
 * is possibly cached in the texture region.
 *
 * \note In reality, this function invalidates the cache tags, forcing the texture cache to load new data. Preloaded textures (see
 * GX_PreloadEntireTexture()) do not use the tags.<br><br>
 *
 * \note The texture hardware can invalidate 4 tags each GP clock. Each tag represents a superline or 512B of TMEM. Therefore, it takes 16
 * GP clocks to invalidate a 32KB texture region.
 *
 * \param[in] region ptr to GXTexRegion object
 *
 * \return none
  *)

procedure GX_InvalidateTexRegion(region: PGXTexRegion); cdecl; external;


(*!
 * \fn void GX_InitTexCacheRegion(GXTexRegion *region,u8 is32bmipmap,u32 tmem_even,u8 size_even,u32 tmem_odd,u8 size_odd)
 * \brief Initializes a texture memory (TMEM) region object for cache.
 *
 * \details The region is allocated by the application and can be used as a cache. An application can create many region objects and some of them can
 * overlap; however, no two overlapping regions can be active at the same time.
 *
 * The possible sizes of a TMEM cache region are 32K, 128K or 512K.
 *
 * \note For pre-loaded textures, the region must be defined by using GX_InitTexPreloadRegion().<br><br>
 *
 * \note GX_Init() creates default texture regions, so it is not necessary for the application to use this function unless a different Texture Memory
 * configuration is desired. In that case, the application should also define a region allocator using GX_SetTexRegionCallback().<br><br>
 *
 * \note The function GX_InvalidateTexRegion() can be used to force the texture in main memory associated with this region to be reloaded. This will be
 * necessary whenever the texture data in main memory changes. You may invalidate all cached regions at once using GX_InvalidateTexAll().
 *
 * \param[in] region ptr to a GXTexRegion struct
 * \param[in] is32bmipmap should be set to <tt>GX_TRUE</tt> to interpret parameters according to the 32b mipmap meaning.
 * \param[in] tmem_even base ptr in TMEM for even LODs; must be multiple of 2KB
 * \param[in] size_even even \ref texcachesize other than <tt>GX_TEXCACHE_NONE</tt>
 * \param[in] tmem_odd base ptr in TMEM for odd LODs; must be multiple of 2KB
 * \param[in] size_odd odd \ref texcachesize other than <tt>GX_TEXCACHE_NONE</tt>
 *
 * \return none
  *)

procedure GX_InitTexCacheRegion(region: PGXTexRegion; is32bmipmap: cuint8;
  tmem_even: cuint32; size_even: cuint8; tmem_odd: cuint32; size_odd: cuint8); cdecl; external;


(*!
 * \fn void GX_InitTexPreloadRegion(GXTexRegion *region,u32 tmem_even,u32 size_even,u32 tmem_odd,u32 size_odd)
 * \brief Initializes a Texture Memory (TMEM) region object for preloading.
 *
 * \details The region is allocated in TMEM by the application and can be used only as a pre-loaded buffer. Cache regions must be allocated
 * by using GX_InitTexCacheRegion(). For pre-loaded textures, the size of the region must match the size of the texture. An application can
 * create many region objects and some of them can overlap; however, no two overlapping regions can be active at the same time.
 *
 * \note The maximum size of a region is 512K.
 *
 * \warning GX_Init() creates no region for preloading, so the application should allocate appropriate regions if preloading is necessary. It
 * is also required to create cache regions and its allocator by using GX_InitTexCacheRegion() and GX_SetTexRegionCallback(), otherwise new
 * cache regions may overwrite the preloaded areas. (Alternatively, if you do not use any color-index textures, you may preload textures into
 * the portion of texture memory normally allocated to color-index usage by the default allocator.)
 *
 * \param[in] region ptr to a GXTexRegion struct
 * \param[in] tmem_even base ptr in TMEM for even LODs; must be 32B aligned
 * \param[in] size_even size of the even cache, in bytes; should be multiple of 32B
 * \param[in] tmem_odd base ptr in TMEM for odd LODs; must be 32B aligned
 * \param[in] size_off size of the odd cache, in bytes; should be multiple of 32B
 *
 * \return none
  *)

procedure GX_InitTexPreloadRegion(region: PGXTexRegion;
  tmem_even, size_even, tmem_odd, size_odd: cuint32); cdecl; external;


(*!
 * \fn void GX_InitTexObj(GXTexObj *obj,void *img_ptr,u16 wd,u16 ht,u8 fmt,u8 wrap_s,u8 wrap_t,u8 mipmap)
 * \brief Used to initialize or change a texture object for non-color index textures.
 *
 * \details Texture objects are used to describe all the parameters associated with a texture, including size, format, wrap modes, filter modes,
 * etc. It is the application's responsibility to provide memory for a texture object. Once initialized, a texture object can be associated with
 * one of eight active texture IDs using GX_LoadTexObj().
 *
 * \note To initialize a texture object for color index format textures, use GX_InitTexObjCI().<br><br>
 *
 * \note If the mipmap flag is <tt>GX_TRUE</tt>, then the texture is a mipmap and the texture will be trilerped. If the mipmap flag is <tt>GX_FALSE</tt>, the texture
 * is not a mipmap and the texture will be bilerped. To override the filter modes and other mipmap controls, see GX_InitTexObjLOD().
 *
 * \param[out] obj ptr to a texture object
 * \param[in] img_ptr ptr to the image data for a texture, aligned to 32B
 * \param[in] wd width of the texture, or LOD level 0 for mipmaps; max value is 1024; mipmaps must be a power of two
 * \param[in] ht height of the texture, or LOD level 0 for mipmaps; max value is 1024; mipmaps must be a power of two
 * \param[in] fmt \ref texfmt
 * \param[in] wrap_s texture coordinate wrapping strategy in the S direction; use <tt>GX_CLAMP</tt>, <tt>GX_REPEAT</tt> or <tt>GX_MIRROR</tt>
 * \param[in] wrap_t texture coordinate wrapping strategy in the T direction; use <tt>GX_CLAMP</tt>, <tt>GX_REPEAT</tt> or <tt>GX_MIRROR</tt>
 * \param[in] mipmap trilinear filtering will be used if <tt>GX_TRUE</tt>, otherwise bilinear is used
 *
 * \return none
  *)

procedure GX_InitTexObj(obj: PGXTexObj; img_ptr: pointer; wd, ht: cuint16;
  fmt, wrap_s, wrap_t, mipmap: cuint8); cdecl; external;


(*!
 * \fn void GX_InitTexObjCI(GXTexObj *obj,void *img_ptr,u16 wd,u16 ht,u8 fmt,u8 wrap_s,u8 wrap_t,u8 mipmap,u32 tlut_name)
 * \brief Used to initialize or change a texture object when the texture is color index format.
 *
 * \details Texture objects are used to describe all the parameters associated with a texture, including size, format, wrap modes, filter modes,
 * etc. It is the application's responsibility to provide memory for a texture object. Once initialized, a texture object can be associated with
 * one of eight active texture IDs using GX_LoadTexObj().
 *
 * \note If the \a mipmap flag is <tt>GX_TRUE</tt>, then the texture is a mipmap and the texture will be filtered using the <tt>GX_LIN_MIP_NEAR</tt> filter mode
 * (color index mipmaps cannot use the <tt>GX_LIN_MIP_LIN</tt> or <tt>GX_NEAR_MIP_LIN</tt> mode). If the \a mipmap flag is <tt>GX_FALSE</tt>, the texture is not a mipmap
 * and the texture will be bilerped. To override the filter modes and other mipmap controls, use GX_InitTexObjLOD(). Mipmap textures should
 * set the width and height to a power of two, but mipmaps do not need to be square.<br><br>
 *
 * \note Non-mipmap (planar) textures do not have to be a power of two. However, to use the <tt>GX_REPEAT</tt> or <tt>GX_MIRROR</tt> modes for \a wrap_s and \a wrap_t
 * the width and height, respectively, must be a power of two.<br><br>
 *
 * \note The \a tlut_name is used to indicate which texture lookup table (TLUT) to use for the index to color conversion. To load the TLUT into
 * texture memory, use GX_LoadTlut().
 *
 * \param[in] obj ptr to a texture object
 * \param[in] img_ptr ptr to the image data for a texture, aligned to 32B
 * \param[in] wd width of the texture, or LOD level 0 for mipmaps; max value is 1024; mipmaps must be a power of two
 * \param[in] ht height of the texture, or LOD level 0 for mipmaps; max value is 1024; mipmaps must be a power of two
 * \param[in] fmt \ref texfmt
 * \param[in] wrap_s texture coordinate wrapping strategy in the S direction; use <tt>GX_CLAMP</tt>, <tt>GX_REPEAT</tt> or <tt>GX_MIRROR</tt>
 * \param[in] wrap_t texture coordinate wrapping strategy in the T direction; use <tt>GX_CLAMP</tt>, <tt>GX_REPEAT</tt> or <tt>GX_MIRROR</tt>
 * \param[in] mipmap if <tt>GX_TRUE</tt>, it is a mipmap texture, else it is a planar texture
 * \param[in] tlut_name TLUT name to use for this texture; default texture configuration recognizes \ref tlutname
 *
 * \return none
  *)

procedure GX_InitTexObjCI(obj: PGXTexObj; img_ptr: pointer; wd, ht: cuint16;
  fmt, wrap_s, wrap_t, mipmap: cuint8; tlut_name: cuint32); cdecl; external;


(*!
 * \fn void GX_InitTexObjTlut(GXTexObj *obj,u32 tlut_name)
 * \brief Allows one to modify the TLUT that is associated with an existing texture object.
 *
 * \param[in] obj ptr to a texture object
 * \param[in] tlut_name TLUT name to use for this texture; default texture configuration recognizes \ref tlutname
 *
 * \return none
  *)

procedure GX_InitTexObjTlut(obj: PGXTexObj; tlut_name: cuint32); cdecl; external;


(*!
 * \fn void GX_InitTexObjData(GXTexObj *obj,void *img_ptr)
 * \brief Allows one to modify the image data pointer for an existing texture object.
 *
 * \note The image format and size for the new data must agree with what they were when the texture object was first initialized using
 * GX_InitTexObj() or GX_InitTexObjCI().
 *
 * \param[in] obj ptr to a texture object
 * \param[in] img_ptr ptr to the texture data in main memory
 *
 * \return none
  *)

procedure GX_InitTexObjData(obj: PGXTexObj; img_ptr: pointer); cdecl; external;


(*!
 * \fn void GX_InitTexObjWrapMode(GXTexObj *obj,u8 wrap_s,u8 wrap_t)
 * \brief Allows one to modify the texture coordinate wrap modes for an existing texture object.
 *
 * \param[in] obj ptr to a texture object
 * \param[in] wrap_s texture coordinate wrapping strategy in the S direction; use <tt>GX_CLAMP</tt>, <tt>GX_REPEAT</tt> or <tt>GX_MIRROR</tt>
 * \param[in] wrap_t texture coordinate wrapping strategy in the T direction; use <tt>GX_CLAMP</tt>, <tt>GX_REPEAT</tt> or <tt>GX_MIRROR</tt>
 *
 * \return none
  *)

procedure GX_InitTexObjWrapMode(obj: PGXTexObj; wrap_s, wrap_t: cuint8); cdecl; external;

procedure GX_InitTexObjFilterMode(obj: PGXTexObj; minfilt, magfilt: cuint8); cdecl; external;

procedure GX_InitTexObjMinLOD(obj: PGXTexObj; minlod: f32); cdecl; external;

procedure GX_InitTexObjMaxLOD(obj: PGXTexObj; maxlod: f32); cdecl; external;

procedure GX_InitTexObjLODBias(obj: PGXTexObj; lodbias: f32); cdecl; external;

procedure GX_InitTexObjBiasClamp(obj: PGXTexObj; biasclamp: cuint8); cdecl; external;

procedure GX_InitTexObjEdgeLOD(obj: PGXTexObj; edgelod: cuint8); cdecl; external;

procedure GX_InitTexObjMaxAniso(obj: PGXTexObj; maxaniso: cuint8); cdecl; external;

procedure GX_InitTexObjUserData(obj: PGXTexObj; userdata: pointer); cdecl; external;

(*!
 * \fn void GX_LoadTexObj(GXTexObj *obj,u8 mapid)
 * \brief Loads the state describing a texture into one of eight hardware register sets.
 *
 * \details Before this happens, the texture object \a obj should be initialized using GX_InitTexObj() or GX_InitTexObjCI(). The \a id parameter refers to
 * the texture state register set. Once loaded, the texture can be used in any Texture Environment (TEV) stage using GX_SetTevOrder().
 *
 * \note This function will call the functions set by GX_SetTexRegionCallback() (and GX_SetTlutRegionCallback() if the texture is color-index
 * format) to obtain the texture regions associated with this texture object. These callbacks are set to default functions by GX_Init().
 *
 * \warning If the texture is a color-index texture, you <b>must</b> load the associated TLUT (using GX_LoadTlut()) before calling GX_LoadTexObj().
 *
 * \param[in] obj ptr to a texture object
 * \param[in] mapid \ref texmapid, <tt>GX_TEXMAP0</tt> to <tt>GX_TEXMAP7</tt> only
 *
 * \return none
  *)

procedure GX_LoadTexObj(obj: PGXTexObj; mapid: cuint8); cdecl; external;


(*!
 * \fn void GX_LoadTlut(GXTlutObj *obj,u32 tlut_name)
 * \brief Copies a Texture Look-Up Table (TLUT) from main memory to Texture Memory (TMEM).
 *
 * \details The \a tlut_name parameter is the name of a pre-allocated area of TMEM. The callback function set by GX_SetTlutRegionCallback() converts
 * the \a tlut_name into a \ref GXTlutRegion pointer. The TLUT is loaded in the TMEM region described by this pointer. The TLUT object \a obj describes the
 * location of the TLUT in main memory, the TLUT format, and the TLUT size. \a obj should have been previously initialized using GX_InitTlutObj().
 *
 * \note GX_Init() sets a default callback to convert \a tlut_names from \ref tlutname to \ref GXTlutRegion pointers. The default configuration of
 * TMEM has 20 TLUTs, 16 each 256 entries by 16 bits, and 4 each 1k entries by 16 bits. This configuration can be overriden by calling
 * GX_InitTlutRegion() and GX_InitTexCacheRegion() to allocate TMEM. Then you can define your own region allocation scheme using GX_SetTlutRegionCallback()
 * and GX_SetTexRegionCallback().
 *
 * \param[in] obj ptr to a TLUT object; application must allocate this
 * \param[in] tlut_name \ref tlutname
 *
 * \return none
  *)

procedure GX_LoadTlut(obj: PGXTlutObj; tlut_name: cuint32); cdecl; external;


(*!
 * \fn void GX_LoadTexObjPreloaded(GXTexObj *obj,GXTexRegion *region,u8 mapid)
 * \brief Loads the state describing a preloaded texture into one of eight hardware register sets.
 *
 * \details Before this happens, the texture object \a obj should be initialized using GX_InitTexObj() or GX_InitTexObjCI(). The \a mapid parameter refers to
 * the texture state register set. The texture should be loaded beforehand using GX_PreloadEntireTexture(). Once loaded, the texture can be used in any Texture Environment
 * (TEV) stage using GX_SetTevOrder().
 *
 * \note GX_Init() initially calls GX_SetTevOrder() to make a simple texture pipeline that associates <tt>GX_TEXMAP0</tt> with <tt>GX_TEVSTAGE0</tt>,
 * <tt>GX_TEXMAP1</tt> with <tt>GX_TEVSTAGE1</tt>, etc.<br><br>
 *
 * \note GX_LoadTexObjPreloaded() will not call the functions set by GX_SetTexRegionCallback() (and GX_SetTlutRegionCallback() if the texture is color
 * index format) because the region is set explicitly; however, these callback functions must be aware of all regions that are preloaded. The default
 * callbacks set by GX_Init() assume there are no preloaded regions.
 *
 * \param[in] obj ptr to a texture object
 * \param[in] region ptr to a region object that describes an area of texture memory
 * \param[in] mapid \ref texmapid for reference in a TEV stage
 *
 * \return none
  *)

procedure GX_LoadTexObjPreloaded(obj: PGXTexObj; region: PGXTexRegion;
  mapid: cuint8); cdecl; external;


(*!
 * \fn void GX_PreloadEntireTexture(GXTexObj *obj,GXTexRegion *region)
 * \brief Loads a given texture from DRAM into the texture memory.
 *
 * \details Accesses to this texture will bypass the texture cache tag look-up and instead read the texels directly from texture memory. The
 * texture region must be the same size as the texture (see GX_InitTexPreloadRegion()).
 *
 * \note This function loads the texture into texture memory, but to use it as a source for the Texture Environment (TEV) unit, you must first
 * call GX_LoadTexObjPreloaded(). The default configuration (as set by GX_Init()) of texture memory has no preloaded regions, so you must install
 * your own region allocator callbacks using GX_SetTexRegionCallback() and GX_SetTlutRegionCallback().
 *
 * \param[in] obj ptr to object describing the texture to laod
 * \param[in] region TMEM texture region to load the texture into
 *
 * \return none
  *)

procedure GX_PreloadEntireTexture(obj: PGXTexObj; region: PGXTexRegion); cdecl; external;


(*!
 * \fn void GX_InitTlutObj(GXTlutObj *obj,void *lut,u8 fmt,u16 entries)
 * \brief Initializes a Texture Look-Up Table (TLUT) object.
 *
 * \details The TLUT object describes the location of the TLUT in main memory, its format and the number of entries. The TLUT in main
 * memory described by this object can be loaded into a TLUT allocated in the texture memory using the GX_LoadTlut() function.
 *
 * \param[in] obj ptr to a TLUT object
 * \param[in] lut ptr to look-up table data; must be 32B aligned
 * \param[in] fmt format of the entries in the TLUt; <tt>GX_TL_IA8</tt>, <tt>GX_TL_RGB565</tt> or <tt>GX_TL_RGB5A3</tt>
 * \param[in] entries number of entries in this table; maximum is 16,384
 *
 * \return none
  *)

procedure GX_InitTlutObj(obj: PGXTlutObj; lut: pointer; fmt: cuint8;
  entries: cuint16); cdecl; external;


(*!
 * \fn void GX_InitTlutRegion(GXTlutRegion *region,u32 tmem_addr,u8 tlut_sz)
 * \brief Initializes a Texture Look-Up Table (TLUT) region object.
 *
 * \note GX_Init() creates default TLUT regions, so the application does not need to call this function unless a new configuration
 * of Texture Memory is desired. In that case, the application should also set a new TLUT region allocator using GX_SetTlutRegionCallback().
 *
 * \param[in] region obj ptr to a TLUT region struct; application must allocate this
 * \param[in] tmem_addr location of the TLU in TMEM; ptr must be aligned to table size
 * \param[in] tlut_sz size of the table
 *
 * \return none
  *)

procedure GX_InitTlutRegion(region: PGXTlutRegion; tmem_addr: cuint32;
  tlut_sz: cuint8); cdecl; external;


(*!
 * \fn void GX_InitTexObjLOD(GXTexObj *obj,u8 minfilt,u8 magfilt,f32 minlod,f32 maxlod,f32 lodbias,u8 biasclamp,u8 edgelod,u8 maxaniso)
 * \brief Sets texture Level Of Detail (LOD) controls explicitly for a texture object.
 *
 * \details It is the application's responsibility to provide memory for a texture object. When initializing a texture object using GX_InitTexObj()
 * or GX_InitTexObjCI(), this information is set to default values based on the mipmap flag. This function allows the programmer to override those
 * defaults.
 *
 * \note This function should be called after GX_InitTexObj() or GX_InitTexObjCI() for a particular texture object.<br><br>
 *
 * \note Setting \a biasclamp prevents over-biasing the LOD when the polygon is perpendicular to the view direction.<br><br>
 *
 * \note \a edgelod should be set if \a biasclamp is set or \a maxaniso is set to <tt>GX_ANISO_2</tt> or <tt>GX_ANISO_4</tt>.<br><br>
 *
 * \note Theoretically, there is no performance difference amongst various magnification/minification filter settings except <tt>GX_LIN_MIP_LIN</tt> filter with
 * <tt>GX_TF_RGBA8</tt> texture format which takes twice as much as other formats. However, this argument is assuming an environment where texture cache always
 * hits. On real environments, you will see some performance differences by changing filter modes (especially minification filter) because cache-hit ratio
 * changes according to which filter mode is being used.
 *
 * \param[in] obj ptr to a texture object
 * \param[in] minfilt \ref texfilter to use when the texel/pixel ratio is >= 1.0
 * \param[in] magfilt \ref texfilter to use when the texel/pixel ratio is < 1.0; use only <tt>GX_NEAR</tt> or <tt>GX_LINEAR</tt>
 * \param[in] minlod minimum LOD value from 0.0 - 10.0 inclusive
 * \param[in] maxlod maximum LOD value from 0.0 - 10.0 inclusive
 * \param[in] lodbias bias to add to computed LOD value
 * \param[in] biasclamp if <tt>GX_ENABLE</tt>, clamp (LOD+lodbias) so that it is never less than the minimum extent of the pixel projected in texture space
 * \param[in] edgelod if <tt>GX_ENABLE</tt>, compute LOD using adjacent texels
 * \param[in] maxaniso \ref anisotropy to use
 *
 * \return none
  *)

procedure GX_InitTexObjLOD(obj: PGXTexObj; minfilt, magfilt: cuint8;
  minlod, maxlod, lodbias: f32; biasclamp, edgelod, maxaniso: cuint8); cdecl; external;


(*!
 * \fn void GX_SetTexCoordScaleManually(u8 texcoord,u8 enable,u16 ss,u16 ts)
 * \brief Overrides the automatic texture coordinate scaling (based upon the associated map size) and lets one manually assign the scale values that
 * are used for a given \a texcoord.
 *
 * \details Setting the \a enable parameter to <tt>GX_TRUE</tt> gives this behavior. The given \a texcoord retains these manual scale values until this function is
 * called again. This function is also used to return a given texture coordinate back to normal, automatic scaling (by setting \a enable to <tt>GX_FALSE</tt>).
 *
 * \note A texture coordinate is scaled after being computed by the relevant texgen and before the actual texture lookup  Normally, the scale value is set
 * according to the texture map that is associated with the texcoord by GX_SetTevOrder(). However, there are certain cases where a different scale value is
 * desirable. One such case is when using indirect tiled textures (see GX_SetTevIndTile()).
 *
 * \param[in] texcoord the \ref texcoordid being changed
 * \param[in] enable if <tt>GX_TRUE</tt>, scale will be set manually, otherwise set automatically and \a ss and \a ts ignored
 * \param[in] ss manual scale value for the S component of the coordinate
 * \param[in] ts manual scale value for the T component of the coordinate
 *
 * \return none
  *)

procedure GX_SetTexCoordScaleManually(texcoord, enable: cuint8; ss, ts: cuint16); cdecl; external;


(*!
 * \fn void GX_SetTexCoordBias(u8 texcoord,u8 s_enable,u8 t_enable)
 * \brief Sets the texture coordinate bias of a particular texture.
 *
 * \details Range bias is used with texture coordinates applied in <tt>GX_REPEAT</tt> wrap mode in order to increase the precision of texture coordinates
 * that spread out over a large range. The texture coordinate values for a primitive are biased (by an equal integer) towards zero early in the
 * graphics pipeline, thus preserving bits for calculation later in the pipe.  Since the coordinates are repeated, this bias by an integer should
 * have no effect upon the actual appearance of the texture.
 *
 * \note Texture coordinate range bias is something that is normally set automatically by the GX API (during GX_Begin()); however, when a texture
 * coordinate is being scaled manually (by using GX_SetTexCoordScaleManually()), the associated bias is no longer modified by GX. Thus,
 * GX_SetTexCoordBias() allows the bias to be changed while a texture coordinate is being manually controlled.
 *
 * \param[in] texcoord \ref texcoordid being changed
 * \param[in] s_enable enable or disable range bias in the S direction with <tt>GX_ENABLE</tt>/<tt>GX_DISABLE</tt>
 * \param[in] t_enable enable or disable range bias in the T direction with <tt>GX_ENABLE</tt>/<tt>GX_DISABLE</tt>
 *
 * \return none
  *)

procedure GX_SetTexCoordBias(texcoord, s_enable, t_enable: cuint8); cdecl; external;


(*!
 * \fn GXTexRegionCallback GX_SetTexRegionCallback(GXTexRegionCallback cb)
 * \brief Sets the callback function called by GX_LoadTexObj() to obtain an available texture region.
 *
 * \details GX_Init() calls this function to set a default region-assignment policy. A programmer can override this default region assignment
 * by implementing his own callback function. A pointer to the texture object and the texture map ID that are passed
 * to GX_LoadTexObj() are provided to the callback function.
 *
 * \param[in] cb ptr to a function that takes a pointer to a GXTexObj and a \ref texmapid as a parameter and returns a pointer to a \ref GXTexRegion.
 *
 * \return pointer to the previously set callback
  *)

function GX_SetTexRegionCallback(cb: GXTexRegionCallback): GXTexRegionCallback; cdecl; external;


(*!
 * \fn GXTlutRegionCallback GX_SetTlutRegionCallback(GXTlutRegionCallback cb)
 * \brief Sets the callback function called by GX_LoadTlut() to find the region into which to load the TLUT.
 *
 * \details GX_LoadTexObj() will also call \a cb to obtain the Texture Look-up Table (TLUT) region when the texture forma
 * is color-index.
 *
 * GX_Init() calls GX_SetTlutRegionCallback() to set a default TLUT index-to-region mapping. The name for the TLUT from the texture
 * object is provided as an argument to the callback. The callback should return a pointer to the \ref GXTlutRegion for this TLUT index.
 *
 * \note For a given \a tlut_name (in the \ref GXTlutRegionCallback struct), \a cb must always return the same \ref GXTlutRegion; this is because
 * GX_LoadTlut() will initialize data into the \ref GXTlutRegion which GX_LoadTexObj() will subsequently use.
 *
 * \param[in] cb ptr to a function that takes a u32 TLUT name as a parameter and returns a pointer to a \ref GXTlutRegion.
 *
 * \return pointer to the previously set callback
  *)

function GX_SetTlutRegionCallback(cb: GXTlutRegionCallback)
  : GXTlutRegionCallback; cdecl; external;


(*!
 * \fn void GX_InitLightPos(GXLightObj *lit_obj,f32 x,f32 y,f32 z)
 * \brief Sets the position of the light in the light object.
 *
 * \details The GameCube graphics hardware supports local diffuse lights. The position of the light should be in the same space as a transformed
 * vertex position (i.e., view space).
 *
 * \note Although the hardware doesn't support parallel directional diffuse lights, it is possible to get "almost parallel" lights by setting
 * sufficient large values to position parameters (x, y and z) which makes the light position very far away from objects to be lit and all rays
 * considered almost parallel.<br><br>
 *
 * \note The memory for the light object must be allocated by the application; this function does not load any hardware registers directly. To
 * load a light object into a hardware light, use GX_LoadLightObj() or GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to the light object
 * \param[in] x X coordinate to place the light at
 * \param[in] y Y coordinate to place the light at
 * \param[in] z Z coordinate to place the light at
 *
 * \return none
  *)

procedure GX_InitLightPos(lit_obj: PGXLightObj; x, y, z: f32); cdecl; external;


(*!
 * \fn void GX_InitLightColor(GXLightObj *lit_obj,GXColor col)
 * \brief Sets the color of the light in the light object.
 *
 * \note The memory for the light object should be allocated by the application; this function does not load any hardware register directly.  To
 * load a light object into a hardware light, use GX_LoadLightObj() or GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to the light object
 * \param[in] col color to set the light to
 *
 * \return none
  *)

procedure GX_InitLightColor(lit_obj: PGXLightObj; col: GXColor); cdecl; external;


(*!
 * \fn void GX_InitLightDir(GXLightObj *lit_obj,f32 nx,f32 ny,f32 nz)
 * \brief Sets the direction of a light in the light object.
 *
 * \details This direction is used when the light object is used as spotlight or a specular light (see the <i>attn_fn</i> parameter of GX_SetChanCtrl()).
 *
 * \note The coordinate space of the light normal should be consistent with a vertex normal transformed by a normal matrix; i.e., it should be
 * transformed to view space.<br><br>
 *
 * \note This function does not set the direction of parallel directional diffuse lights. If you want parallel diffuse lights, you may put the light
 * position very far from every objects to be lit. (See GX_InitLightPos() and GX_SetChanCtrl())<br><br>
 *
 * \note The memory for the light object must be allocated by the application; this function does not load any hardware registers.  To load a light
 * object into a hardware light, use GX_LoadLightObj() or GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to the light object
 * \param[in] nx X coordinate of the light normal
 * \param[in] ny Y coordinate of the light normal
 * \param[in] nz Z coordinate of the light normal
 *
 * \return none
  *)

procedure GX_InitLightDir(lit_obj: PGXLightObj; nx, ny, nz: f32); cdecl; external;


(*!
 * \fn void GX_LoadLightObj(GXLightObj *lit_obj,u8 lit_id)
 * \brief Loads a light object into a set of hardware registers associated with a \ref lightid.
 *
 * \details This function copies the light object data into the graphics FIFO through the CPU write-gather buffer mechanism. This guarantees that
 * the light object is coherent with the CPU cache.
 *
 * \note The light object must have been initialized first using the necessary GX_InitLight*() functions.<br><br>
 *
 * \note Another way to load a light object is with GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to the light object to load
 * \param[in] lit_id \ref lightid to load this light into
 *
 * \return none
  *)

procedure GX_LoadLightObj(lit_obj: PGXLightObj; lit_id: cuint8); cdecl; external;


(*!
 * \fn void GX_LoadLightObjIdx(u32 litobjidx,u8 litid)
 * \brief Instructs the GP to fetch the light object at \a ltobjindx from an array.
 *
 * \details The light object is retrieved from the array to which <tt>GX_SetArray(GX_VA_LIGHTARRAY, ...)</tt> points. Then it loads the object into
 * the hardware register associated with \ref lightid.
 *
 * \note Data flows directly from the array in DRAM to the GP; therefore, the light object data may not be coherent with the CPU's cache. The
 * application is responsible for storing the light object data from the CPU cache (using DCStoreRange()) before calling GX_LoadLightObjIdx().
 *
 * \param[in] litobjidx index to a light object
 * \param[in] litid \ref lightid to load this light into
 *
 * \return none
  *)

procedure GX_LoadLightObjIdx(litobjidx: cuint32; litid: cuint8); cdecl; external;


(*!
 * \fn void GX_InitLightDistAttn(GXLightObj *lit_obj,f32 ref_dist,f32 ref_brite,u8 dist_fn)
 * \brief Sets coefficients for distance attenuation in a light object.
 *
 * \details This function uses three easy-to-control parameters instead of <i>k0</i>, <i>k1</i>, and <i>k2</i> in GX_InitLightAttn().
 *
 * In this function, you can specify the brightness on an assumed reference point. The parameter \a ref_distance is distance between the light
 * and the reference point. The parameter \a ref_brite specifies ratio of the brightness on the reference point. The value for \a ref_dist should
 * be greater than 0 and that for \a ref_brite should be within 0 < \a ref_brite < 1, otherwise distance attenuation feature is turned off. The
 * parameter \a dist_fn defines type of the brightness decreasing curve by distance; <tt>GX_DA_OFF</tt> turns distance attenuation feature off.
 *
 * \note If you want more flexible control, it is better to use GX_InitLightAttn() and calculate appropriate coefficients.<br><br>
 *
 * \note This function sets parameters only for distance attenuation. Parameters for angular attenuation should be set by using
 * GX_InitLightSpot() or GX_InitLightAttnA().<br><br>
 *
 * \note This function does not load any hardware registers directly. To load a light object into a hardware light, use GX_LoadLightObj() or
 * GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to a light object
 * \param[in] ref_dist distance between the light and reference point
 * \param[in] red_brite brightness of the reference point
 * \param[in] dist_fn \ref distattnfn to use
 *
 * \return none
  *)

procedure GX_InitLightDistAttn(lit_obj: PGXLightObj; ref_dist, ref_brite: f32;
  dist_fn: cuint8); cdecl; external;


(*!
 * \fn void GX_InitLightAttn(GXLightObj *lit_obj,f32 a0,f32 a1,f32 a2,f32 k0,f32 k1,f32 k2)
 * \brief Sts coefficients used in the lighting attenuation calculation in a given light object.
 *
 * \details The parameters \a a0, \a a1, and \a a2 are used for angular (spotlight) attenuation. The coefficients \a k0, \a k1, and \a k2 are used for
 * distance attenuation. The attenuation function is:
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<i>atten</i> = <i>clamp0</i>(\a a2^2 * <i>aattn</i>^2 + \a a1 * <i>aattn</i> + \a a0) / (\a k2 * <i>d</i>^2 + \a k1  * <i>d</i> + \a k0)
 *
 * where <i>aattn</i> is the cosine of the angle between the light direction and the vector from the light position to the vertex, and <i>d</i> is
 * the distance from the light position to the vertex when the channel attenuation function is <tt>GX_AF_SPOT</tt>. The light color will be
 * multiplied by the <i>atten</i> factor when the attenuation function for the color channel referencing this light is set to <tt>GX_AF_SPOT</tt>
 * (see GX_SetChanCtrl()).
 *
 * \note The convenience function GX_InitLightSpot() can be used to set the angle attenuation coefficents based on several spot light
 * types. The convenience function GX_InitLightDistAttn() can be used to set the distance attenuation coefficients using one of several
 * common attenuation functions.<br><br>
 *
 * \note The convenience macro GX_InitLightShininess() can be used to set the attenuation parameters for specular lights.<br><br>
 *
 * \note When the channel attenuation function is set to <tt>GX_AF_SPEC</tt>, the <i>aattn</i> and <i>d</i> parameter are equal to the dot product of the
 * eye-space vertex normal and the half-angle vector set by GX_InitSpecularDir().<br><br>
 *
 * \note This function does not load any hardware registers directly. To load a light object into a hardware light, use GX_LoadLightObj()
 * or GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to a light object
 * \param[in] a0 angle attenuation coefficient
 * \param[in] a1 angle attenuation coefficient
 * \param[in] a2 angle attenuation coefficient
 * \param[in] k0 distance attenuation coefficient
 * \param[in] k1 distance attenuation coefficient
 * \param[in] k2 distance attenuation coefficient
 *
 * \return none
  *)

procedure GX_InitLightAttn(lit_obj: PGXLightObj; a0, a1, a2, k0, k1, k2: f32); cdecl; external;


(*!
 * \fn void GX_InitLightAttnA(GXLightObj *lit_obj,f32 a0,f32 a1,f32 a2)
 * \brief Sets coefficients used in the lighting angle attenuation calculation in a given light object.
 *
 * \details The parameters \a a0, \a a1, and \a a2 are used for angular (spotlight) attenuation. The attenuation
 * function is:
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<i>atten</i> = <i>clamp0</i>(\a a2^2 * <i>cos(theta)</i>^2 + \a a1 * <i>cos(theta)</i> + \a a0) / (\a k2 * <i>d</i>^2 + \a k1  * <i>d</i> + \a k0)
 *
 * where <i>cos(theta)</i> is the cosine of the angle between the light normal and the vector from the light position to the vertex, and <i>d</i> is the distance
 * from the light position to the vertex. The \a k0-\a 2 coefficients can be set using GX_InitLightAttnK(). You can set both the \a a0-\a 2 and \a k0-\a 2 coefficients
 * can be set using GX_InitLightAttn(). The light color will be multiplied by the <i>atten</i> factor when the attenuation function for the color channel
 * referencing this light is set to <tt>GX_AF_SPOT</tt> (see GX_SetChanCtrl()).
 *
 * \note The convenience function GX_InitLightSpot() can be used to set the angle attenuation coefficents based on several spot light types. The
 * convenience function GX_InitLightDistAttn() can be used to set the distance attenuation coefficients using one of several common attenuation functions.<br><br>
 *
 * \note This function does not load any hardware registers directly. To load a light object into a hardware light, use GX_LoadLightObj() or GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to a light object
 * \param[in] a0 angle attenuation coefficient
 * \param[in] a1 angle attenuation coefficient
 * \param[in] a2 angle attenuation coefficient
 *
 * \return none
  *)

procedure GX_InitLightAttnA(lit_obj: PGXLightObj; a0, a1, a2: f32); cdecl; external;


(*!
 * \fn void GX_InitLightAttnK(GXLightObj *lit_obj,f32 k0,f32 k1,f32 k2)
 * \brief Sets coefficients used in the lighting distance attenuation calculation in a given light object.
 *
 * \details The coefficients \a k0, \a k1, and \a k2 are used for distance attenuation. The attenuation function is:
 *
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<i>atten</i> = <i>clamp0</i>(\a a2^2 * <i>cos(theta)</i>^2 + \a a1 * <i>cos(theta)</i> + \a a0) / (\a k2 * <i>d</i>^2 + \a k1  * <i>d</i> + \a k0)
 *
 * where <i>cos(theta)</i> is the cosine of the angle between the light normal and the vector from the light position to the vertex, and <i>d</i> is the distance
 * from the light position to the vertex. The \a a0-\a 2 coefficients can be set using GX_InitLightAttnA(). You can set both the \a a0-\a 2 and \a k0-\a 2 coefficients
 * can be set using GX_InitLightAttn(). The light color will be multiplied by the <i>atten</i> factor when the attenuation function for the color channel
 * referencing this light is set to <tt>GX_AF_SPOT</tt> (see GX_SetChanCtrl()).
 *
 * \note The convenience function GX_InitLightSpot() can be used to set the angle attenuation coefficents based on several spot light types. The convenience
 * function GX_InitLightDistAttn() can be used to set the distance attenuation coefficients using one of several common attenuation functions.<br><br>
 *
 * \note Note that this function does not load any hardware registers directly. To load a light object into a hardware light, use GX_LoadLightObj() or
 * GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to a light object
 * \param[in] k0 distance attenuation coefficient
 * \param[in] k1 distance attenuation coefficient
 * \param[in] k2 distance attenuation coefficient
 *
 * \return none
  *)

procedure GX_InitLightAttnK(lit_obj: PGXLightObj; k0, k1, k2: f32); cdecl; external;


(*!
 * \fn void GX_InitSpecularDirHA(GXLightObj *lit_obj,f32 nx,f32 ny,f32 nz,f32 hx,f32 hy,f32 hz)
 * \brief Sets the direction and half-angle vector of a specular light in the light object.
 *
 * \detail These vectors are used when the light object is used only as specular light. In contrast to GX_InitSpecularDir(),
 * which caclulates half-angle vector automatically by assuming the view vector as (0, 0, 1), this function allows users to
 * specify half-angle vector directly as input arguments. It is useful to do detailed control for orientation of highlights.
 *
 * \note This function does not load any hardware registers. To load a light object into a hardware light, use GX_LoadLightObj()
 * or GX_LoadLightObjIdx().<br><br>
 *
 * \note Other notes are similar to those described in GX_InitSpecularDir().
 *
 * \param[in] lit_obj ptr to a light object
 * \param[in] nx X coordinate of the light normal
 * \param[in] ny Y coordinate of the light normal
 * \param[in] nz Z coordinate of the light normal
 * \param[in] hx X coordinate of half-angle
 * \param[in] hy Y coordinate of half-angle
 * \param[in] hz Z coordinate of half-angle
 *
 * \return none
  *)

procedure GX_InitSpecularDirHA(lit_obj: PGXLightObj;
  nx, ny, nz, hx, hy, hz: f32); cdecl; external;


(*!
 * \fn void GX_InitSpecularDir(GXLightObj *lit_obj,f32 nx,f32 ny,f32 nz)
 * \brief Sets the direction of a specular light in the light object.
 *
 * \details This direction is used when the light object is used only as specular light. The coordinate space of the light normal
 * should be consistent with a vertex normal transformed by a normal matrix; i.e., it should be transformed to view space.
 *
 * \note This function should be used if and only if the light object is used as specular light. One specifies a specular light in
 * GX_SetChanCtrl() by setting the \ref attenfunc to <tt>GX_AF_SPEC</tt>. Furthermore, one must not use GX_InitLightDir() or
 * GX_InitLightPos() to set up a light object which will be used as a specular light since these functions will destroy the information
 * set by GX_InitSpecularDir(). In contrast to diffuse lights (including spotlights) that are considered local lights, a specular light
 * is a parallel light (i.e. the specular light is infinitely far away such that all the rays of the light are parallel), and thus one
 * can only specify directional information.
 *
 * \note This function does not load any hardware registers. To load a light object into a hardware light, use GX_LoadLightObj()
 * or GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to a light object
 * \param[in] nx X coordinate of the light normal
 * \param[in] ny Y coordinate of the light normal
 * \param[in] nz Z coordinate of the light normal
 *
 * \return none
  *)

procedure GX_InitSpecularDir(lit_obj: PGXLightObj; nx, ny, nz: f32); cdecl; external;


(*!
 * \fn void GX_InitLightSpot(GXLightObj *lit_obj,f32 cut_off,u8 spotfn)
 * \brief Sets coefficients for angular (spotlight) attenuation in light object.
 *
 * \details This function uses two easy-to-control parameters instead of \a a0, \a a1, and \a a2 on GX_InitLightAttn().
 *
 * \details The parameter \a cut_off specifies cutoff angle of the spotlight by degree. The spotlight works while the angle between the ray for a vertex and
 * the light direction given by GX_InitLightDir() is smaller than this cutoff angle. The value for \a cut_off should be within 0 < \a cut_off <= 90.0, otherwise
 * given light object doesn't become a spotlight.
 *
 * The parameter \a spotfn defines type of the illumination distribution within cutoff angle. The value <tt>GX_SP_OFF</tt> turns spotlight feature off even if
 * color channel setting is using <tt>GX_AF_SPOT</tt> (see GX_SetChanCtrl()).
 *
 * \note This function can generate only some kind of simple spotlights. If you want more flexible control, it is better to use GX_InitLightAttn() and calculate
 * appropriate coefficients.<br><br>
 *
 * \note This function sets parameters only for angular attenuation. Parameters for distance attenuation should be set by using GX_InitLightDistAttn() or
 * GX_InitLightAttnK().<br><br>
 *
 * \note This function does not load any hardware registers directly. To load a light object into a hardware light, use GX_LoadLightObj() or GX_LoadLightObjIdx().
 *
 * \param[in] lit_obj ptr to a light object
 * \param[in] cut_off cutoff angle of the spotlight, in degrees
 * \param[in] spotfn \ref spotfn to use for this light
 *
 * \return none
  *)

procedure GX_InitLightSpot(lit_obj: PGXLightObj; cut_off: f32; spotfn: cuint8); cdecl; external;


function GX_ReadClksPerVtx: cuint32; cdecl; external;

function GX_GetOverflowCount: cuint32; cdecl; external;

function GX_ResetOverflowCount: cuint32; cdecl; external;


(*!
 * \fn lwp_t GX_GetCurrentGXThread()
 * \brief Returns the current GX thread.
 *
 * \details The current GX thread should be the thread that is currently responsible for generating graphics data. By default,
 * the GX thread is the thread that invoked GX_Init(); however, it may be changed by calling GX_SetCurrentGXThread().
 *
 * \note When graphics data is being generated in immediate mode (that is, the CPU FIFO = GP FIFO, and the GP is actively consuming
 * data), the high watermark may be triggered. When this happens, the high watermark interrupt handler will suspend the GX thread, thus
 * preventing any further graphics data from being generated. The low watermark interrupt handler will resume the thread.
 *
 * \return the current GX thread
  *)

function GX_GetCurrentGXThread: lwp_t; cdecl; external;


(*!
 * \fn lwp_t GX_SetCurrentGXThread()
 * \brief Sets the current GX thread to the calling thread.
 *
 * \details The new thread should be the thread that will be responsible for generating graphics data. By default, the GX thread is
 * the thread that invoked GX_Init(); however, it may be changed by calling this function.
 *
 * \note It is a programming error to change GX thread while the current GX thread is suspended by a high water mark interrupt. This
 * indicates that you have two threads about to generate GX data.<br><br>
 *
 * \note When graphics data is being generated in immediate mode (that is, the CPU FIFO = GP FIFO, and the GP is actively consuming
 * data), the high watermark may be triggered. When this happens, the high watermark interrupt handler will suspend the GX thread, thus
 * preventing any further graphics data from being generated. The low watermark interrupt handler will resume the thread.
 *
 * \return the previous GX thread ID
  *)

function GX_SetCurrentGXThread: lwp_t; cdecl; external;


(*!
 * \fn void GX_RestoreWriteGatherPipe()
 * \brief Restores the write-gather pipe.
 *
 * \details The CPU fifo that was attached at the time GX_RedirectWriteGatherPipe() was called will be re-attached. If there is data pending
 * in the write gather pipe (e.g. if the amount of data written was not a multiple of 32 bytes), the data will be padded with zeroes and
 * flushed out.
 *
 * \warning This function must be called between successive calls to GX_RedirectWriteGatherPipe().
 *
 * \return none
  *)

procedure GX_RestoreWriteGatherPipe; cdecl; external;


(*!
 * \fn void GX_SetGPMetric(u32 perf0,u32 perf1)
 * \brief Sets two performance metrics to measure in the GP.
 *
 * \details perf0 and perf1 are set to measure. The initial metrics measured are <tt>GX_PERF0_NONE</tt> and <tt>GX_PERF1_NONE</tt>, which return counts of zero
 * for the first call to GX_ReadGPMetric().
 *
 * Each performance counter has a unique set of events or ratios that it can count. In some cases the same metric can be counted using both
 * counters, for example <tt>GX_PERF0_VERTICES</tt> and <tt>GX_PERF1_VERTICES</tt>. Ratios (the metric name ends in <tt>_RATIO</tt>) are multiplied by
 * 1000 (1000 = all misses/clips, etc., 0 = no misses/clips, etc.).
 *
 * \note GX_ReadGPMetric() and GX_ClearGPMetric() can be used in the callback associated with the draw sync interrupt (see GX_SetDrawSyncCallback()).
 * This function should not be used in the draw sync callback because it will insert tokens in the GP command stream at random times.
 *
 * \warning This function reads results from CPU-accessible registers in the GP, therefore, this command <i>must not</i> be used in a display list. In
 * addition, the performance counters in some cases are triggered by sending tokens through the Graphics FIFO to the GP.  This implies that
 * the function should only be used in immediate mode (when the Graphics FIFO is connected to the CPU and the GP at the same time).  It may
 * also be necessary to send a draw sync token using GX_SetDrawSync() or call GX_SetDrawDone() after GX_ReadGPMetric() to ensure that the
 * state has actually been processed by the GP.
 *
 * \param[in] perf0 \ref perf0metrics to measure
 * \param[in] perf1 \ref perf1metrics to measure
 *
 * \returns none
  *)

procedure GX_SetGPMetric(perf0, perf1: cuint32); cdecl; external;


(*!
 * \fn void GX_ClearGPMetric()
 * \brief Clears the two virtual GP performance counters to zero.
 *
 * \note The counter's function is set using GX_SetGPMetric(); the counter's value is read using GX_ReadGPMetric(). Consult these for more details.
 *
 * \warning This function resets CPU accessible counters, so it should <b>not</b> be used in a display list.
 *
 * \return none
  *)

procedure GX_ClearGPMetric; cdecl; external;


(*!
 * \fn void GX_InitXfRasMetric()
 * \brief Initialize the transformation unit (XF) rasterizer unit (RAS) to take performance measurements.
 *
 * \warning This function should be avoided; use the GP performance metric functions instead.
 *
 * \return none
  *)

procedure GX_InitXfRasMetric; cdecl; external;


(*!
 * \fn void GX_ReadXfRasMetric(u32 *xfwaitin,u32 *xfwaitout,u32 *rasbusy,u32 *clks)
 * \brief Read performance metric values from the XF and RAS units.
 *
 * \warning This function should be avoided; use the GP performance metric functions instead.<br><br>
 *
 * \warning The parameters for this function are a best guess based on names and existing code.
 *
 * \param[out] xfwaitin Number of clocks the XF has waited for data to arrive?
 * \param[out] xfwaitout Number of clocks the XF has waited to push finished data down?
 * \param[out] rasbusy Number of clocks the RAS has spent being busy?
 * \param[out] clks Clocks that have passed since last count reset?
 *
 * \return none
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure GX_ReadXfRasMetric(var xfwaitin: cuint32; var xfwaitout: cuint32; var rasbusy: cuint32; var clks: cuint32); cdecl; external;
=======
procedure GX_ReadXfRasMetric(xfwaitin, xfwaitout, rasbusy, clks: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadXfRasMetric(xfwaitin, xfwaitout, rasbusy, clks: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadXfRasMetric(xfwaitin, xfwaitout, rasbusy, clks: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadXfRasMetric(xfwaitin, xfwaitout, rasbusy, clks: pcuint32); cdecl; external;
>>>>>>> origin/cpstrnew


(*!
 * \fn void GX_ClearVCacheMetric()
 * \brief Clears the Vertex Cache performance counter.
 *
 * \details This function clears the performance counter by sending a special clear token via the Graphics FIFO.
 *
 * \note To set the metric for the counter, call GX_SetVCacheMetric(); to read the counter value, call GX_ReadVCacheMetric().
 *
 * \return none
  *)

procedure GX_ClearVCacheMetric; cdecl; external;


(*!
 * \fn void GX_ReadVCacheMetric(u32 *check,u32 *miss,u32 *stall)
 * \brief Returns Vertex Cache performance counters.
 *
 * \details Each call to this function resets the counter to zero. GX_SetVCacheMetric() sets the metric to be measured by
 * the Vertex Cache performance counter.
 *
 * \warning This function reads CPU-accessible registers in the GP and so should not be called in a display list.
 *
 * \param[out] check total number of accesses to the vertex cache
 * \param[out] miss total number of cache misses to the vertex cache
 * \param[out] stall number of GP clocks that the vertex cache was stalled
 *
 * \return none
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure GX_ReadVCacheMetric(var check: cuint32; var miss: cuint32; var stall: cuint32); cdecl; external;
=======
procedure GX_ReadVCacheMetric(check, miss, stall: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadVCacheMetric(check, miss, stall: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadVCacheMetric(check, miss, stall: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadVCacheMetric(check, miss, stall: pcuint32); cdecl; external;
>>>>>>> origin/cpstrnew


(*!
 * \fn void GX_SetVCacheMetric(u32 attr)
 * \brief Sets the metric the Vertex Cache performance counter will measure.
 *
 * \details It is possible to monitor a particular attribute or all attributes using \a attr.
 *
 * \note To clear the counter, call GX_ClearVCacheMetric(); to read the counter value, call GX_ReadVCacheMetric().
 *
 * \param[in] attr \ref vcachemetrics to measure
 *
 * \return none
  *)

procedure GX_SetVCacheMetric(attr: cuint32); cdecl; external;


(*!
 * \fn void GX_GetGPStatus(u8 *overhi,u8 *underlow,u8 *readIdle,u8 *cmdIdle,u8 *brkpt)
 * \brief Reads the current status of the GP.
 *
 * \details \a overhi and \a underlow will indicate whether or not the watermarks have been reached. If the CPU and GP FIFOs
 * are the same, then \a overhi will indicate whether or not the current GX thread is suspended. The value of \a brkpt can be
 * used to determine if a breakpoint is in progress (i.e. GP reads are suspended; they are resumed by a call to
 * GX_DisableBreakPt()). A callback can also be used to notify your application that the break point has been reached. (see
 * GX_SetBreakPtCallback())
 *
 * \param[out] overhi <tt>GX_TRUE</tt> if high watermark has been passed
 * \param[out] underlow <tt>GX_TRUE</tt> if low watermark has been passed
 * \param[out] readIdle <tt>GX_TRUE</tt> if the GP read unit is idle
 * \param[out] cmdIdle <tt>GX_TRUE</tt> if all commands have been flushed to XF
 * \param[out] brkpt <tt>GX_TRUE</tt> if FIFO has reached a breakpoint and GP reads have been stopped
 *
 * \return none
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure GX_GetGPStatus(var overhi: cuint32; var underlow: cuint8; var readIdle: cuint8; var cmdIdle: cuint8; var brkpt: cuint8); cdecl; external;
=======
procedure GX_GetGPStatus(overhi, underlow, readIdle, cmdIdle, brkpt: pcuint8); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_GetGPStatus(overhi, underlow, readIdle, cmdIdle, brkpt: pcuint8); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_GetGPStatus(overhi, underlow, readIdle, cmdIdle, brkpt: pcuint8); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_GetGPStatus(overhi, underlow, readIdle, cmdIdle, brkpt: pcuint8); cdecl; external;
>>>>>>> origin/cpstrnew


(*!
 * \fn void GX_ReadGPMetric(u32 *cnt0,u32 *cnt1)
 * \brief Returns the count of the previously set performance metrics.
 *
 * \note The performance metrics can be set using GX_SetGPMetric(); the counters can be cleared using GX_ClearGPMetric().<br><br>
 *
 * \note GX_ReadGPMetric() and GX_ClearGPMetric() can be used in the callback associated with the draw sync interrupt (see GX_SetDrawSyncCallback()).
 * The function GX_SetGPMetric() should <b>not</b> be used in the draw sync callback because it will insert tokens in the GP command stream at random times.<br><br>
 *
 * \warning This function reads results from CPU-accessible registers in the GP, therefore, this command <i>must not</i> be used in a display list. It
 * may also be necessary to send a draw sync token using GX_SetDrawSync() or GX_SetDrawDone() before GX_ReadGPMetric() is called to ensure that the
 * state has actually been processed by the GP.
 *
 * \param[out] cnt0 current value of GP counter 0
 * \param[out] cnt1 current value of GP counter 1
 *
 * \return none
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure GX_ReadGPMetric(var cnt0: cuint32; var cnt1: cuint32); cdecl; external;
=======
procedure GX_ReadGPMetric(cnt0, cnt1: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadGPMetric(cnt0, cnt1: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadGPMetric(cnt0, cnt1: pcuint32); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadGPMetric(cnt0, cnt1: pcuint32); cdecl; external;
>>>>>>> origin/cpstrnew


(*!
 * \fn void GX_ReadBoundingBox(u16 *t,u16 *b,u16 *l,u16 *r)
 * \brief Returns the bounding box of pixel coordinates that are drawn in the Embedded Framebuffer (EFB).
 *
 * \details This function reads the bounding box values. GX_ClearBoundingBox() can be used reset the values of the bounding box.
 *
 * \note Since the hardware can only test the bounding box in quads (2x2 pixel blocks), the result of this function may contain error
 * of plus or minus 1 pixel. Also because of this, <b>left</b> and <b>top</b> are always even-numbered and <b>right</b> and <b>bottom</b>
 * are always odd-numbered.
 *
 * \param[out] top uppermost line in the bounding box
 * \param[out] bottom lowest line in the bounding box
 * \param[out] left leftmost pixel in the bounding box
 * \param[out] right rightmost pixel in the bounding box
 *
 * \return none
  *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure GX_ReadBoundingBox(var top: cuint16; var bottom: cuint16; var left: cuint16; var right: cuint16); cdecl; external;
=======
procedure GX_ReadBoundingBox(top, bottom, left, right: pcuint16); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadBoundingBox(top, bottom, left, right: pcuint16); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadBoundingBox(top, bottom, left, right: pcuint16); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure GX_ReadBoundingBox(top, bottom, left, right: pcuint16); cdecl; external;
>>>>>>> origin/cpstrnew


(*!
 * \fn volatile void* GX_RedirectWriteGatherPipe(void *ptr)
 * \brief Temporarily points the CPU's write-gather pipe at a new location.
 *
 * \details After calling this function, subsequent writes to the address returned by this function (or the WGPipe union)
 * will be gathered and sent to a destination buffer. The write pointer is automatically incremented by the GP.  The write
 * gather pipe can be restored by calling GX_RestoreWriteGatherPipe(). This function cannot be called between a
 * GX_Begin()/GX_End() pair.
 *
 * \note The destination buffer, referred to by \a ptr, must be 32 byte aligned. The amount of data written should
 * also be 32-byte aligned. If it is not, zeroes will be added to pad the destination buffer to 32 bytes. No part of the
 * destination buffer should be modified inside the CPU caches - this may introduce cache incoherency problems.<br><br>
 *
 * \note The write gather pipe is one of the fastest ways to move data out of the CPU (the other being the locked cache DMA).
 * In general, you are compute-bound when sending data from the CPU.<br><br>
 *
 * \note This function is cheaper than trying to create a fake CPU fifo around a destination buffer, which requires calls to
 * GX_SetCPUFifo(), GX_InitFifoBase(), etc. This function performs very light weight state saves by assuming that the CPU and
 * GP FIFOs never change.
 *
 * \warning <b>No GX commands can be called until the write gather pipe is restored. You MUST call
 * GX_RestoreWriteGatherPipe() before calling this function again, or else the final call to restore the pipe will fail.</b>
 *
 * \param[in] ptr to destination buffer, 32-byte aligned
 *
 * \return real address of the write-gather "port". All writes to this address will be gathered by the CPU write gather pipe.
 * You may also use the WGPipe union. If you do not use the WGPipe union, ensure that your local variable is volatile.
  *)

function GX_RedirectWriteGatherPipe(ptr: pointer): pointer; cdecl; external;



procedure GX_InitLightPosv(lo: PGXLightObj; vec: pointer); inline;
procedure GX_InitLightDirv(lo: PGXLightObj; vec: pointer); inline;
procedure GX_InitSpecularDirv(lo: PGXLightObj; vec: pointer); inline;
procedure GX_InitSpecularDirHAv(lo: PGXLightObj; vec0, vec1: pointer); inline;
procedure GX_InitLightShininess(lobj: PGXLightObj; shininess: f32); inline;


{$ENDIF OGC_INTERFACE}



{$IFDEF OGC_IMPLEMENTATION}
procedure GX_End(); inline;
begin
end;

procedure GX_Position3f32(x, y, z: f32); inline;
begin
	wgPipe^.F32 := x;
	wgPipe^.F32 := y;
	wgPipe^.F32 := z;
end;

procedure GX_Position3u16(x, y, z: cuint16); inline;
begin
	wgPipe^.U16 := x;
	wgPipe^.U16 := y;
	wgPipe^.U16 := z;
end;

procedure GX_Position3s16(x, y, z: cint16); inline;
begin
	wgPipe^.S16 := x;
	wgPipe^.S16 := y;
	wgPipe^.S16 := z;
end;

procedure GX_Position3u8(x, y, z: cuint8); inline;
begin
	wgPipe^.U8 := x;
	wgPipe^.U8 := y;
	wgPipe^.U8 := z;
end;

procedure GX_Position3s8(x, y, z: cint8); inline;
begin
	wgPipe^.S8 := x;
	wgPipe^.S8 := y;
	wgPipe^.S8 := z;
end;

procedure GX_Position2f32(x, y: f32); inline;
begin
	wgPipe^.F32 := x;
	wgPipe^.F32 := y;
end;

procedure GX_Position2u16(x, y: cuint16); inline;
begin
	wgPipe^.U16 := x;
	wgPipe^.U16 := y;
end;

procedure GX_Position2s16(x, y: cint16); inline;
begin
	wgPipe^.S16 := x;
	wgPipe^.S16 := y;
end;

procedure GX_Position2u8(x, y: cuint8); inline;
begin
	wgPipe^.U8 := x;
	wgPipe^.U8 := y;
end;

procedure GX_Position2s8(x, y: cint8); inline;
begin
	wgPipe^.S8 := x;
	wgPipe^.S8 := y;
end;

procedure GX_Position1x8(index: cuint8); inline;
begin
	wgPipe^.U8 := index;
end;    

procedure GX_Position1x16(index: cuint16); inline;
begin
	wgPipe^.U16 := index;
end;

procedure GX_Normal3f32(nx, ny, nz: f32); inline;
begin
	wgPipe^.F32 := nx;
	wgPipe^.F32 := ny;
	wgPipe^.F32 := nz;
end;

procedure GX_Normal3s16(nx, ny, nz: cint16); inline;
begin
	wgPipe^.S16 := nx;
	wgPipe^.S16 := ny;
	wgPipe^.S16 := nz;
end;

procedure GX_Normal3s8(nx, ny, nz: cint8); inline;
begin
	wgPipe^.S8 := nx;
	wgPipe^.S8 := ny;
	wgPipe^.S8 := nz;
end;

procedure GX_Normal1x8(index: cuint8); inline;
begin
	wgPipe^.U8 := index;
end;

procedure GX_Normal1x16(index: cuint16); inline;
begin
	wgPipe^.U16 := index;
end;

procedure GX_Color4u8(r, g, b, a: cuint8); inline;
begin
	wgPipe^.U8 := r;
	wgPipe^.U8 := g;
	wgPipe^.U8 := b;
	wgPipe^.U8 := a;
end;

procedure GX_Color3u8(r, g, b: cuint8); inline;
begin
	wgPipe^.U8 := r;
	wgPipe^.U8 := g;
	wgPipe^.U8 := b;
end;

procedure GX_Color3f32(r, g, b: f32); inline;
begin
	wgPipe^.U8 := cuint8(trunc(r * 255.0));
	wgPipe^.U8 := cuint8(trunc(g * 255.0));
	wgPipe^.U8 := cuint8(trunc(b * 255.0));
end;

procedure GX_Color1u32(clr: cuint32); inline;
begin
	wgPipe^.U32 := clr;
end;

procedure GX_Color1u16(clr: cuint16); inline;
begin
	wgPipe^.U16 := clr;
end;

procedure GX_Color1x8(index: cuint8); inline;
begin
	wgPipe^.U8 := index;
end;

procedure GX_Color1x16(index: cuint16); inline;
begin
	wgPipe^.U16 := index;
end;

procedure GX_TexCoord2f32(s, t: f32); inline;
begin
	wgPipe^.F32 := s;
	wgPipe^.F32 := t;
end;

procedure GX_TexCoord2u16(s, t: cuint16); inline;
begin
	wgPipe^.U16 := s;
	wgPipe^.U16 := t;
end;

procedure GX_TexCoord2s16(s, t: cint16); inline;
begin
	wgPipe^.S16 := s;
	wgPipe^.S16 := t;
end;

procedure GX_TexCoord2u8(s, t: cuint8); inline;
begin
	wgPipe^.U8 := s;
	wgPipe^.U8 := t;
end;

procedure GX_TexCoord2s8(s, t: cint8); inline;
begin
	wgPipe^.S8 := s;
	wgPipe^.S8 := t;
end;

procedure GX_TexCoord1f32(s: f32); inline;
begin
	wgPipe^.F32 := s;
end;

procedure GX_TexCoord1u16(s: cuint16); inline;
begin
	wgPipe^.U16 := s;
end;

procedure GX_TexCoord1s16(s: cint16); inline;
begin
	wgPipe^.S16 := s;
end;

procedure GX_TexCoord1u8(s: cuint8); inline;
begin
	wgPipe^.U8 := s;
end;

procedure GX_TexCoord1s8(s: cint8); inline;
begin
	wgPipe^.S8 := s;
end;

procedure GX_TexCoord1x8(index: cuint8); inline;
begin
	wgPipe^.U8 := index;
end;

procedure GX_TexCoord1x16(index: cuint16); inline;
begin
	wgPipe^.U16 := index;
end;

procedure GX_MatrixIndex1x8(index: cuint8); inline;
begin
	wgPipe^.U8 := index;
end;

procedure GX_InitLightPosv(lo: PGXLightObj; vec: pointer); inline;
begin
  GX_InitLightPos(lo, f32(pointer(vec)^), f32(pointer(vec + 1)^), f32(pointer(vec + 2)^));
end;


procedure GX_InitLightDirv(lo: PGXLightObj; vec: pointer); inline;
begin
  GX_InitLightDir(lo, f32(pointer(vec)^), f32(pointer(vec + 1)^), f32(pointer(vec + 2)^));
end;

procedure GX_InitSpecularDirv(lo: PGXLightObj; vec: pointer); inline;
begin
  GX_InitSpecularDir(lo, f32(pointer(vec)^), f32(pointer(vec + 1)^), f32(pointer(vec + 2)^));
end;

procedure GX_InitSpecularDirHAv(lo: PGXLightObj; vec0, vec1: pointer); inline;
begin
  GX_InitSpecularDirHA(lo,
    f32(pointer(vec0)^), f32(pointer(vec0 + 1)^), f32(pointer(vec0 + 2)^),
    f32(pointer(vec1)^), f32(pointer(vec1 + 1)^), f32(pointer(vec1 + 2)^));
end;


procedure GX_InitLightShininess(lobj: PGXLightObj; shininess: f32); inline;
begin
  GX_InitLightAttn(lobj, 0.0, 0.0, 1.0, shininess / 2.0, 0.0, 1.0 - shininess / 2.0 );
end;

{$ENDIF OGC_IMPLEMENTATION}


