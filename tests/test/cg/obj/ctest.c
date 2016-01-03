/*
  Program to test linking between C and pascal units.
  Copyright (c) 2002, Carl Eric Codere
*/

/*
   Note : Arrays seem to always be passed by reference
   in the C language. Therefore, no testing is required
   to use them.
*/

<<<<<<< HEAD
<<<<<<< HEAD
#if defined(__BORLANDC__)
#define long_long __int64
#else
#define long_long long long
#endif

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
unsigned char global_u8bit;
unsigned short global_u16bit;
unsigned int global_u32bit;
short global_s16bit;
int global_s32bit;
<<<<<<< HEAD
<<<<<<< HEAD
long_long global_s64bit;
unsigned long_long global_u64bit;
=======
long long global_s64bit;
unsigned long long global_u64bit;
>>>>>>> graemeg/fixes_2_2
=======
long long global_s64bit;
unsigned long long global_u64bit;
>>>>>>> origin/fixes_2_2
float global_float;
double global_double;
long double global_long_double;

#define   RESULT_U8BIT    0x55
#define   RESULT_U16BIT   0x500F
#define   RESULT_U32BIT   0x500F0000
#define   RESULT_S16BIT   -12
#define   RESULT_S32BIT   -120
#define   RESULT_S64BIT   -12000
#define   RESULT_U64BIT   0x1BCDABCD
#define   RESULT_PCHAR    "Hello world"
#define   RESULT_FLOAT       14.54
#define   RESULT_DOUBLE      15.54
#define	  RESULT_LONGDOUBLE  16.54


struct _1BYTE_
{
	unsigned char  u8;
};

struct _3BYTE_
{
	unsigned char  u8;
	unsigned short u16;
};

struct _3BYTE_S
{
	unsigned short u16;
	unsigned char w8;
};

struct _5BYTE_
{
	unsigned char  u8;
	unsigned int u32;
};

struct _7BYTE_
{
	unsigned char u8;
<<<<<<< HEAD
<<<<<<< HEAD
	long_long s64;
=======
	long long s64;
>>>>>>> graemeg/fixes_2_2
=======
	long long s64;
>>>>>>> origin/fixes_2_2
	unsigned short u16;
};


struct _7BYTE_ test_struct;


/* simple parameter testing */
void test_param_u8(unsigned char v)
{
  global_u8bit = v;
}


void test_param_u16(unsigned short v)
{
  global_u16bit = v;
}

void test_param_u32(unsigned int v)
{
  global_u32bit = v;
}


void test_param_s16(short v)
{
  global_s16bit = v;
}

void test_param_s32(int v)
{
  global_s32bit = v;
}


<<<<<<< HEAD
<<<<<<< HEAD
void test_param_s64(long_long v)
=======
void test_param_s64(long long v)
>>>>>>> graemeg/fixes_2_2
=======
void test_param_s64(long long v)
>>>>>>> origin/fixes_2_2
{
  global_s64bit = v;
}

<<<<<<< HEAD
<<<<<<< HEAD
void test_param_u64(unsigned long_long v)
=======
void test_param_u64(unsigned long long v)
>>>>>>> graemeg/fixes_2_2
=======
void test_param_u64(unsigned long long v)
>>>>>>> origin/fixes_2_2
{
  global_u64bit = v;
}

void test_param_float(float v)
{
  global_float = v;
}

void test_param_double(double v)
{
  global_double = v;
}


void test_param_longdouble(long double v)
{
  global_long_double = v;
}

/* simple array parameter testing */
void test_array_param_u8(unsigned char v[2])
{
  global_u8bit = v[1];
}


void test_array_param_u16(unsigned short v[2])
{
  global_u16bit = v[1];
}

void test_array_param_u32(unsigned int v[2])
{
  global_u32bit = v[1];
}


void test_array_param_s16(short v[2])
{
  global_s16bit = v[1];
}

void test_array_param_s32(int v[2])
{
  global_s32bit = v[1];
}


<<<<<<< HEAD
<<<<<<< HEAD
void test_array_param_s64(long_long v[2])
=======
void test_array_param_s64(long long v[2])
>>>>>>> graemeg/fixes_2_2
=======
void test_array_param_s64(long long v[2])
>>>>>>> origin/fixes_2_2
{
  global_s64bit = v[1];
}

<<<<<<< HEAD
<<<<<<< HEAD
void test_array_param_u64(unsigned long_long v[2])
=======
void test_array_param_u64(unsigned long long v[2])
>>>>>>> graemeg/fixes_2_2
=======
void test_array_param_u64(unsigned long long v[2])
>>>>>>> origin/fixes_2_2
{
  global_u64bit = v[1];
}

void test_array_param_float(float v[2])
{
  global_float = v[1];
}

void test_array_param_double(double v[2])
{
  global_double = v[1];
}


void test_array_param_longdouble(long double v[2])
{
  global_long_double = v[1];
}

/* if this one works, others should also automatically */
void test_param_var_u8(unsigned char *x)
{
	*x = RESULT_U8BIT;
}

/* mixed parameter testing */
void test_param_mixed_u16(unsigned char z, unsigned short x, unsigned char y)
{
	global_u16bit = x;
	global_u8bit = y;
}

void test_param_mixed_u32(unsigned char z, unsigned int x, unsigned char y)
{
	global_u32bit = x;
	global_u8bit = y;
}

<<<<<<< HEAD
<<<<<<< HEAD
void test_param_mixed_s64(unsigned char z, long_long x, unsigned char y)
=======
void test_param_mixed_s64(unsigned char z, long long x, unsigned char y)
>>>>>>> graemeg/fixes_2_2
=======
void test_param_mixed_s64(unsigned char z, long long x, unsigned char y)
>>>>>>> origin/fixes_2_2
{
	global_s64bit = x;
	global_u8bit = y;
}

void test_param_mixed_var_u8(unsigned char *x, unsigned char y)
{
	global_u8bit = y;
	*x = RESULT_U8BIT;
}

/* mixed parameter testing with floating point args */
void test_param_mixed_float(float x, unsigned char y)
{
	global_float = x;
	global_u8bit = y;
}

void test_param_mixed_double(double x, unsigned char y)
{
	global_double = x;
	global_u8bit = y;
}

void test_param_mixed_long_double(long double x, unsigned char y)
{
	global_long_double = x;
	global_u8bit = y;
}

/* simple record testing */
void test_param_struct_tiny(struct _1BYTE_ buffer)
{
	global_u8bit = buffer.u8;
}

void test_param_struct_small(struct _3BYTE_ buffer)
{
	global_u8bit = buffer.u8;
	global_u16bit = buffer.u16;
}
void test_param_struct_small_s(struct _3BYTE_S buffer)
{
	global_u8bit = buffer.w8;
	global_u16bit = buffer.u16;
}

void test_param_struct_medium(struct _5BYTE_ buffer)
{
	global_u8bit = buffer.u8;
	global_u32bit = buffer.u32;
}

void test_param_struct_large(struct _7BYTE_ buffer)
{
	global_u8bit = buffer.u8;
	global_u16bit = buffer.u16;
	global_s64bit = buffer.s64;
}


/* record+char testing */
void test_param_mixed_struct_tiny(struct _1BYTE_ buffer, unsigned char y)
{
	global_u8bit = y;
}

void test_param_mixed_struct_small(struct _3BYTE_ buffer, unsigned char y)
{
	global_u8bit = y;
	global_u16bit = buffer.u16;
}
void test_param_mixed_struct_small_s(struct _3BYTE_S buffer, unsigned char y)
{
	global_u8bit = y;
	global_u16bit = buffer.u16;
}

void test_param_mixed_struct_medium(struct _5BYTE_ buffer, unsigned char y)
{
        global_u8bit = y;
	global_u32bit = buffer.u32;
}

void test_param_mixed_struct_large(struct _7BYTE_ buffer, unsigned char y)
{
        global_u8bit = y;
	global_u16bit = buffer.u16;
	global_s64bit = buffer.s64;
}


/* function result testing */
unsigned char test_function_u8()
{
	return RESULT_U8BIT;
}

unsigned short test_function_u16()
{
	return RESULT_U16BIT;
}

unsigned int test_function_u32()
{
	return RESULT_U32BIT;
}

<<<<<<< HEAD
<<<<<<< HEAD
unsigned long_long test_function_u64()
=======
unsigned long long test_function_u64()
>>>>>>> graemeg/fixes_2_2
=======
unsigned long long test_function_u64()
>>>>>>> origin/fixes_2_2
{
	return RESULT_U64BIT;
}

unsigned short test_function_s16()
{
	return RESULT_S16BIT;
}

unsigned int test_function_s32()
{
	return RESULT_S32BIT;
}

<<<<<<< HEAD
<<<<<<< HEAD
unsigned long_long test_function_s64()
=======
unsigned long long test_function_s64()
>>>>>>> graemeg/fixes_2_2
=======
unsigned long long test_function_s64()
>>>>>>> origin/fixes_2_2
{
	return RESULT_S64BIT;
}

char* test_function_pchar()
{
	return RESULT_PCHAR;
}

float test_function_float()
{
	return RESULT_FLOAT;
}

double test_function_double()
{
	return RESULT_DOUBLE;
}

long double test_function_longdouble()
{
	return RESULT_LONGDOUBLE;
}

struct _1BYTE_ test_function_tiny_struct()
{
        struct _1BYTE_ test_struct;
	test_struct.u8 = RESULT_U8BIT;
	return test_struct;
}

struct _3BYTE_ test_function_small_struct()
{
        struct _3BYTE_ test_struct;
	test_struct.u8 = RESULT_U8BIT;
	test_struct.u16 = RESULT_U16BIT;
	return test_struct;
}
struct _3BYTE_S test_function_small_struct_s()
{
        struct _3BYTE_S test_struct;
	test_struct.u16 = RESULT_U16BIT;
	test_struct.w8 = RESULT_U8BIT;
	return test_struct;
}

struct _5BYTE_ test_function_medium_struct()
{
        struct _5BYTE_ test_struct;
	test_struct.u8 = RESULT_U8BIT;
	test_struct.u32 = RESULT_U32BIT;
	return test_struct;
}

struct _7BYTE_ test_function_struct()
{
	test_struct.u8 = RESULT_U8BIT;
	test_struct.s64 = RESULT_S64BIT;
	test_struct.u16 = RESULT_U16BIT;
	return test_struct;
}
