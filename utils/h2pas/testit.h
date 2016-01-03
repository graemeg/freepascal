/*
   Test header file to test conversion program.
*/

typedef struct {
  int x;
  int y;
  } a;

typedef union fpk {
  int X;
  int y;
  int z;
} b;

typedef _test test;

struct _test
{
  int x;
  int y;
};

struct XML_cp {
  enum XML_Content_Type         type;
  enum XML_Content_Quant        quant;
  struct _test                  test;
  union _test2                  test2;
  XML_Char *                    name;
  unsigned int                  numchildren;
  XML_Content *                 children;
};

typedef void (*XML_AttlistDeclHandler) (void           *userData,
                                        const XML_Char *elname,
                                        const XML_Char *attname,
                                        const XML_Char *att_type,
                                        const XML_Char *dflt,
                                        int             isrequired);
void proc(int *,int);
void proc(int *p,int i);

float f();

typedef enum { First, second, third } C;

typedef enum { DFirst = 1, DSecond = 2, DThird = 3 } D;

typedef enum { EFirst = 100, ESecond, EThird } D;

void someproc(char *Firstarg,...);

mytype* somefunc (char *firstarg);

#define test 0x012345UL

extern long long i641;
extern unsigned long long q641;
extern long long int i642;
extern unsigned long long int q642;

typedef DWORD (WINAPI *LPTHREAD_START_ROUTINE)(LPVOID);
typedef DWORD(WINAPI *LPPROGRESS_ROUTINE)(LARGE_INTEGER,LARGE_INTEGER,LARGE_INTEGER,LARGE_INTEGER,DWORD,DWORD,HANDLE,HANDLE,LPVOID);

typedef Status (*XcmsConversionProc)();

typedef XrmHashTable XrmSearchList[];

#define XrmStringToRepresentation(string)   XrmStringToQuark(string)
#define XrmRepresentationToString(type)   XrmQuarkToString(type)

typedef struct _XRenderPictureAttributes {
    Bool                repeat;
    Picture             alpha_map;
    int                 alpha_x_origin;
    int                 alpha_y_origin;
    int                 clip_x_origin;
    int                 clip_y_origin;
    Pixmap              clip_mask;
    Bool                graphics_exposures;
    int                 subwindow_mode;
    int                 poly_edge;
    int                 poly_mode;
    Atom                dither;
} XRenderPictureAttributes;

void   gdk_gc_set_dashes          (GdkGC            *gc,
                                   gint              dash_offset,
                                   gint8             dash_list[],
                                   gint              n);
<<<<<<< HEAD
<<<<<<< HEAD

typedef struct FnTable {
  int (*Fn1)( void );
  int (*Fn2)( void );
  int (*Fn3)( void );
} FnTable;


void f(int a, char* p[]);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
