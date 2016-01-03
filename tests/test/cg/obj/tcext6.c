<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#if defined(__BORLANDC__)
/* ## include <stdint.h> */
#define long_long __int64
#define int8_t __int8
#define uint8_t  unsigned __int8
#define int16_t __int16
#define uint16_t  unsigned __int16
#define int32_t __int32
#define uint32_t  unsigned __int32
#define int64_t __int64
#define uint64_t  unsigned __int64

#else
#include <stdint.h>
#define long_long long long
#endif
=======
#include <stdint.h>
>>>>>>> graemeg/cpstrnew
=======
#include <stdint.h>
>>>>>>> graemeg/cpstrnew
=======
#include <stdint.h>
>>>>>>> graemeg/cpstrnew
=======
#include <stdint.h>
>>>>>>> origin/cpstrnew

struct struct1 {
  float v;
  };

struct struct2 {
  double v;
  };


struct struct3 {
  float v1;
  float v2;
  };

struct struct4 {
  double v1;
  float v2;
  };

struct struct5 {
  double v1;
  double v2;
  };

struct struct6 {
  double v1;
  float v2;
  float v3;
  };

struct struct7 {
  float v1;
  int32_t v2;
  double v3;
  };

struct struct8 {
  union {
    float v1;
    double d;
    };
  };

struct struct9 {
  int64_t v1;
  float v2;
  };

struct struct10 {
  int64_t v1;
  int16_t v2;
  float v3;
  };

struct struct11 {
  int64_t v1;
  double v2;
  };

struct struct12 {
  int64_t v1;
  float v2;
  float v3;
  };

struct struct13 {
  double v1;
  int64_t v2;
  };

struct struct14 {
  double v1;
  int32_t v2;
  int16_t v3;
  };

struct struct15 {
  double v1;
  int32_t v2;
  float v3;
  };

struct struct16 {
  float v1;
  float v2;
  float v3;
  float v4;
  };

struct struct17 {
  float v1;
  double v2;
  };

struct struct31 {
  long double v1;
  float v2;
  };


/* to avoid depending on libc for double->int64 conversions

License/credit:

Written by John R. Hauser.  This work was made possible in part by the
International Computer Science Institute, located at Suite 600, 1947 Center
Street, Berkeley, California 94704.  Funding was partially provided by the
National Science Foundation under grant MIP-9311980.  The original version
of this code was written as part of a project to build a fixed-point vector
processor in collaboration with the University of California at Berkeley,
overseen by Profs. Nelson Morgan and John Wawrzynek.  More information
is available through the Web page `http://www.cs.berkeley.edu/~jhauser/
arithmetic/SoftFloat.html'.

THIS SOFTWARE IS DISTRIBUTED AS IS, FOR FREE.  Although reasonable effort has
been made to avoid it, THIS SOFTWARE MAY CONTAIN FAULTS THAT WILL AT TIMES
RESULT IN INCORRECT BEHAVIOR.  USE OF THIS SOFTWARE IS RESTRICTED TO PERSONS
AND ORGANIZATIONS WHO CAN AND WILL TAKE FULL RESPONSIBILITY FOR ALL LOSSES,
COSTS, OR OTHER PROBLEMS THEY INCUR DUE TO THE SOFTWARE, AND WHO FURTHERMORE
EFFECTIVELY INDEMNIFY JOHN HAUSER AND THE INTERNATIONAL COMPUTER SCIENCE
INSTITUTE (possibly via similar legal warning) AGAINST ALL LOSSES, COSTS, OR
OTHER PROBLEMS INCURRED BY THEIR CUSTOMERS AND CLIENTS DUE TO THE SOFTWARE.

Derivative works are acceptable, even for commercial purposes, so long as
(1) the source code for the derivative work includes prominent notice that
the work is derivative, and (2) the source code includes prominent notice with
these four paragraphs for those parts of this code that are retained.


*/

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#define LIT64(a) a##LL

#define double2float64(a) (*(float64*)&(a))
=======
#define LIT64( a ) a##LL

#define double2float64( a ) (*(float64*)&(a))
>>>>>>> graemeg/cpstrnew
=======
#define LIT64( a ) a##LL

#define double2float64( a ) (*(float64*)&(a))
>>>>>>> graemeg/cpstrnew
=======
#define LIT64( a ) a##LL

#define double2float64( a ) (*(float64*)&(a))
>>>>>>> graemeg/cpstrnew
=======
#define LIT64( a ) a##LL

#define double2float64( a ) (*(float64*)&(a))
>>>>>>> origin/cpstrnew

typedef char flag;

typedef int64_t float64;
typedef uint64_t bits64;
typedef int64_t sbits64;

bits64 extractFloat64Frac( float64 a )
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    return a & LIT64(0x000FFFFFFFFFFFFF) ;
=======
    return a & LIT64( 0x000FFFFFFFFFFFFF );
>>>>>>> graemeg/cpstrnew
=======
    return a & LIT64( 0x000FFFFFFFFFFFFF );
>>>>>>> graemeg/cpstrnew
=======
    return a & LIT64( 0x000FFFFFFFFFFFFF );
>>>>>>> graemeg/cpstrnew
=======
    return a & LIT64( 0x000FFFFFFFFFFFFF );
>>>>>>> origin/cpstrnew
}

int16_t extractFloat64Exp( float64 a )
{
    return ( a>>52 ) & 0x7FF;
}

flag extractFloat64Sign( float64 a )
{
    return a>>63;
}

int32_t int64_is_zero(bits64 a0)
{
  return (((uint32_t)(a0 >> 32)) == 0) && ((((uint32_t)(a0 & LIT64(0xFFFFFFFF)))) == 0);
}

void shift64ExtraRightJamming(bits64 a0, bits64 a1, int16_t count, bits64 *z0Ptr, bits64 *z1Ptr )
{
    bits64 z0, z1;
    int8_t negCount = ( - count ) & 63;

    if ( count == 0 ) {
        z1 = a1;
        z0 = a0;
    }
    else if ( count < 64 ) {
        z1 = ( a0<<negCount ) | ( !int64_is_zero(a1));
        z0 = a0>>count;
    }
    else {
        if ( count == 64 ) {
            z1 = a0 | ( !int64_is_zero(a1) );
        }
        else {
            z1 = ( !int64_is_zero( a0 | a1 ) );
        }
        z0 = 0;
    }
    *z1Ptr = z1;
    *z0Ptr = z0;
}

static int64_t roundAndPackInt64( flag zSign, bits64 absZ0, bits64 absZ1 )
{
    int64_t z;

    z = absZ0;
    if ( zSign ) z = - z;
    return z;

}

int64_t float64_to_int64( float64 a )
{
    flag aSign;
    int16_t aExp, shiftCount;
    bits64 aSig, aSigExtra;

    aSig = extractFloat64Frac( a );
    aExp = extractFloat64Exp( a );
    aSign = extractFloat64Sign( a );
    if ( aExp ) aSig |= LIT64( 0x0010000000000000 );
    shiftCount = 0x433 - aExp;
    if ( shiftCount <= 0 ) {
        if ( 0x43E < aExp ) {
            if (    ! aSign
                 || (    ( aExp == 0x7FF )
                      && ( aSig != LIT64( 0x0010000000000000 ) ) )
               ) {
                return LIT64( 0x7FFFFFFFFFFFFFFF );
            }
            return (sbits64) LIT64( 0x8000000000000000 );
        }
        aSigExtra = 0;
        aSig <<= - shiftCount;
    }
    else {
        shift64ExtraRightJamming( aSig, 0, shiftCount, &aSig, &aSigExtra );
    }
    return roundAndPackInt64( aSign, aSig, aSigExtra );
}

float pass1(struct struct1 s) {
  return s.v;
}

double pass2(struct struct2 s) {
  return s.v;
}

float pass3(struct struct3 s) {
  return s.v1 + s.v2;
}

double pass4(struct struct4 s) {
  return s.v1 + s.v2;
}

double pass5(struct struct5 s) {
  return s.v1 + s.v2;
}

double pass6(struct struct6 s) {
  return s.v1 + s.v2;
}

double pass7(struct struct7 s) {
  return s.v1 + s.v2 + s.v3;
}

double pass8(struct struct8 s) {
  return s.d;
}

int64_t pass9(struct struct9 s) {
  double d2 = s.v2;
  return s.v1 + float64_to_int64(double2float64(d2));
}

int64_t pass10(struct struct10 s) {
  double d3 = s.v3;
  return s.v1 + s.v2 + float64_to_int64(double2float64(d3));
}

int64_t pass11(struct struct11 s) {
  double d2 = s.v2;
  return s.v1 + float64_to_int64(double2float64(d2));
}

int64_t pass12(struct struct12 s) {
  double d2 = s.v2;
  double d3 = s.v3;
  return s.v1 + float64_to_int64(double2float64(d2)) + float64_to_int64(double2float64(d3));
}

int64_t pass13(struct struct13 s) {
  double d1 = s.v1;
  return float64_to_int64(double2float64(d1)) + s.v2;
}

int64_t pass14(struct struct14 s) {
  double d1 = s.v1;
  return float64_to_int64(double2float64(d1)) + s.v2 + s.v3;
}

int64_t pass15(struct struct15 s) {
  double d1 = s.v1;
  double d3 = s.v3;
  return float64_to_int64(double2float64(d1)) + s.v2 + float64_to_int64(double2float64(d3));
}

float pass16(struct struct16 s) {
  return s.v1 + s.v2 + s.v3 + s.v4;
}

float pass17(struct struct17 s) {
  return s.v1 + s.v2;
}

long double pass31(struct struct31 s, char b, float *v2) {
  *v2 = s.v2;
  return s.v1;
}



struct struct1 pass1a(char b, struct struct1 s) {
  return s;
}

struct struct2 pass2a(char b, struct struct2 s) {
  return s;
}

struct struct3 pass3a(char b, struct struct3 s) {
  return s;
}

struct struct4 pass4a(char b, struct struct4 s) {
  return s;
}

struct struct5 pass5a(char b, struct struct5 s) {
  return s;
}

struct struct6 pass6a(char b, struct struct6 s) {
  return s;
}

struct struct7 pass7a(char b, struct struct7 s) {
  return s;
}

struct struct8 pass8a(char b, struct struct8 s) {
  return s;
}

struct struct9 pass9a(char b, struct struct9 s) {
  return s;
}

struct struct10 pass10a(char b, struct struct10 s) {
  return s;
}

struct struct11 pass11a(char b, struct struct11 s) {
  return s;
}

struct struct12 pass12a(char b, struct struct12 s) {
  return s;
}

struct struct13 pass13a(char b, struct struct13 s) {
  return s;
}

struct struct14 pass14a(char b, struct struct14 s) {
  return s;
}

struct struct15 pass15a(char b, struct struct15 s) {
  return s;
}

struct struct16 pass16a(char b, struct struct16 s) {
  return s;
}

struct struct17 pass17a(char b, struct struct17 s) {
  return s;
}

struct struct31 pass31a(char b, struct struct31 s) {
  return s;
}
