{
    This file is part of the Numlib package.
    Copyright (c) 1986-2000 by
     Kees van Ginneken, Wil Kortsmit and Loek van Reij of the
     Computational centre of the Eindhoven University of Technology

    FPC port Code          by Marco van de Voort (marco@freepascal.org)
             documentation by Michael van Canneyt (Michael@freepascal.org)

    Special functions. (Currently, most of these functions only work for
            ArbFloat=REAL/DOUBLE, not for Extended(at least not at full
            precision, implement the tables in the diverse functions
            for extended)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit spe;
{$I DIRECT.INC}

interface

uses typ;

{  Calculate modified Besselfunction "of the first kind" I0(x) }
function spebi0(x: ArbFloat): ArbFloat;

{  Calculate modified Besselfunction "of the first kind" I1(x) }
function spebi1(x: ArbFloat): ArbFloat;

{  Calculate Besselfunction "of the first kind" J0(x) }
function spebj0(x: ArbFloat): ArbFloat;

{  Calculate Besselfunction "of the first kind" J1(x) }
function spebj1(x: ArbFloat): ArbFloat;

{  Calculate modified Besselfunction "of the second kind" K0(x) }
function spebk0(x: ArbFloat): ArbFloat;

{  Calculate modified Besselfunction "of the second kind" K1(x) }
function spebk1(x: ArbFloat): ArbFloat;

{  Calculate Besselfunction "of the second kind" Y0(x) }
function speby0(x: ArbFloat): ArbFloat;

{  Calculate Besselfunction "of the second kind" Y1(x) }
function speby1(x: ArbFloat): ArbFloat;

{  Entier function, calculates first integer greater or equal than X}
function speent(x: ArbFloat): longint;

{  Errorfunction ( 2/sqrt(pi)* Int(t,0,pi,exp(sqr(t)) )}
function speerf(x: ArbFloat): ArbFloat;

{  Errorfunction's complement ( 2/sqrt(pi)* Int(t,pi,inf,exp(sqr(t)) )}
function speefc(x: ArbFloat): ArbFloat;

{  Function to calculate the Gamma function ( int(t,0,inf,t^(x-1)*exp(-t)) }
function spegam(x: ArbFloat): ArbFloat;

{  Function to calculate the natural logaritm of the Gamma function}
function spelga(x: ArbFloat): ArbFloat;

{  "Calculates" the maximum of two ArbFloat values     }
function spemax(a, b: ArbFloat): ArbFloat;

{  Calculates the functionvalue of a polynomalfunction with n coefficients in a
for variable X }
function spepol(x: ArbFloat; var a: ArbFloat; n: ArbInt): ArbFloat;

{ Calc a^b with a and b real numbers}
function spepow(a, b: ArbFloat): ArbFloat;

{ Returns sign of x (-1 for x<0, 0 for x=0 and 1 for x>0)  }
function spesgn(x: ArbFloat): ArbInt;

{  ArcSin(x) }
function spears(x: ArbFloat): ArbFloat;

{  ArcCos(x) }
function spearc(x: ArbFloat): ArbFloat;

{  Sinh(x) }
function spesih(x: ArbFloat): ArbFloat;

{  Cosh(x) }
function specoh(x: ArbFloat): ArbFloat;

{  Tanh(x) }
function spetah(x: ArbFloat): ArbFloat;

{  ArcSinH(x) }
function speash(x: ArbFloat): ArbFloat;

{  ArcCosh(x) }
function speach(x: ArbFloat): ArbFloat;

{  ArcTanH(x) }
function speath(x: ArbFloat): ArbFloat;

implementation

function spebi0(x: ArbFloat): ArbFloat;

const

     xvsmall = 3.2e-9;
          a1 : array[0..23] of ArbFloat =
               (  3.08508322553671039e-1, -1.86478066609466760e-1,
                  1.57686843969995904e-1, -1.28895621330524993e-1,
                  9.41616340200868389e-2, -6.04316795007737183e-2,
                  3.41505388391452157e-2, -1.71317947935716536e-2,
                  7.70061052263382555e-3, -3.12923286656374358e-3,
                  1.15888319775791686e-3, -3.93934532072526720e-4,
                  1.23682594989692688e-4, -3.60645571444886286e-5,
                  9.81395862769787105e-6, -2.50298975966588680e-6,
                  6.00566861079330132e-7, -1.36042013507151017e-7,
                  2.92096163521178835e-8, -5.94856273204259507e-9,
                  1.13415934215369209e-9, -2.10071360134551962e-10,
                  4.44484446637868974e-11,-7.48150165756234957e-12);
          a2 : array[0..26] of ArbFloat =
               (  1.43431781856850311e-1, -3.71571542566085323e-2,
                  1.44861237337359455e-2, -6.30121694459896307e-3,
                  2.89362046530968701e-3, -1.37638906941232170e-3,
                  6.72508592273773611e-4, -3.35833513200679384e-4,
                  1.70524543267970595e-4, -8.74354291104467762e-5,
                  4.48739019580173804e-5, -2.28278155280668483e-5,
                  1.14032404021741277e-5, -5.54917762110482949e-6,
                  2.61457634142262604e-6, -1.18752840689765504e-6,
                  5.18632519069546106e-7, -2.17653548816447667e-7,
                  8.75291839187305722e-8, -3.34900221934314738e-8,
                  1.24131668344616429e-8, -4.66215489983794905e-9,
                  1.58599776268172290e-9, -3.80370174256271589e-10,
                  1.23188158175419302e-10,-8.46900307934754898e-11,
                  2.45185252963941089e-11);
           a3: array[0..19] of ArbFloat =
               (  4.01071065066847416e-1,  2.18216817211694382e-3,
                  5.59848253337377763e-5,  2.79770701849785597e-6,
                  2.17160501061222148e-7,  2.36884434055843528e-8,
                  3.44345025431425567e-9,  6.47994117793472057e-10,
                  1.56147127476528831e-10, 4.82726630988879388e-11,
                  1.89599322920800794e-11, 1.05863621425699789e-11,
                  8.27719401266046976e-12, 2.82807056475555021e-12,
                 -4.34624739357691085e-12,-4.29417106720584499e-12,
                  4.30812577328136192e-13, 1.44572313799118029e-12,
                  4.73229306831831040e-14,-1.95679809047625728e-13);


var t : ArbFloat;

begin
  t:=abs(x);
  if t <=xvsmall
  then
    spebi0:=1
  else
  if t <= 4
  then
    spebi0 := exp(t)*spepol(t/2-1, a1[0], SizeOf(a1) div SizeOf(ArbFloat) -1)
  else
  if t <= 12
  then
    spebi0:=exp(t)*spepol(t/4-2, a2[0], SizeOf(a2) div SizeOf(ArbFloat) -1)
  else { t > 12}
    spebi0:=(exp(t)/sqrt(t))*
            spepol(24/t-1, a3[0], SizeOf(a3) div SizeOf(ArbFloat) -1)
end; {spebi0}

function spebi1(x: ArbFloat): ArbFloat;


const xvsmall = 3.2e-9;
      a1: array[0..11] of ArbFloat =
      ( 1.19741654963670236e+0, 9.28758890114609554e-1,
        2.68657659522092832e-1, 4.09286371827770484e-2,
        3.84763940423809498e-3, 2.45224314039278904e-4,
        1.12849795779951847e-5, 3.92368710996392755e-7,
        1.06662712314503955e-8, 2.32856921884663846e-10,
        4.17372709788222413e-12,6.24387910353848320e-14);

      a2: array[0..26] of ArbFloat =
      ( 1.34142493292698178e-1, -2.99140923897405570e-2,
        9.76021102528646704e-3, -3.40759647928956354e-3,
        1.17313412855965374e-3, -3.67626180992174570e-4,
        8.47999438119288094e-5,  5.21557319070236939e-6,
       -2.62051678511418163e-5,  2.47493270133518925e-5,
       -1.79026222757948636e-5,  1.13818992442463952e-5,
       -6.63144162982509821e-6,  3.60186151617732531e-6,
       -1.83910206626348772e-6,  8.86951515545183908e-7,
       -4.05456611578551130e-7,  1.76305222240064495e-7,
       -7.28978293484163628e-8,  2.84961041291017650e-8,
       -1.07563514207617768e-8,  4.11321223904934809e-9,
       -1.41575617446629553e-9,  3.38883570696523350e-10,
       -1.10970391104678003e-10, 7.79929176497056645e-11,
       -2.27061376122617856e-11);

       a3: array[0..19] of ArbFloat =
       ( 3.92624494204116555e-1, -6.40545360348237412e-3,
        -9.12475535508497109e-5, -3.82795135453556215e-6,
        -2.72684545741400871e-7, -2.82537120880041703e-8,
        -3.96757162863209348e-9, -7.28107961041827952e-10,
        -1.72060490748583241e-10,-5.23524129533553498e-11,
        -2.02947854602758139e-11,-1.11795516742222899e-11,
        -8.69631766630563635e-12,-3.05957293450420224e-12,
         4.42966462319664333e-12, 4.47735589657057690e-12,
        -3.95353303949377536e-13,-1.48765082315961139e-12,
        -5.77176811730370560e-14, 1.99448557598015488e-13);

var t : ArbFloat;

begin
  t:=abs(x);
  if t <= xvsmall
  then
    spebi1:=x/2
  else
  if t <= 4
  then
    spebi1:=x*spepol(sqr(t)/8-1, a1[0], sizeof(a1) div sizeof(ArbFloat)-1)
  else
  if t <= 12
  then
    spebi1:=
      exp(t)*spepol(t/4-2, a2[0], sizeof(a2) div sizeof(ArbFloat)-1)*spesgn(x)
  else { t > 12}
    spebi1:=
      (exp(t)/sqrt(t))*
      spepol(24/t-1, a3[0], sizeof(a3) div sizeof(ArbFloat)-1)*spesgn(x)
end; {spebi1}

function spebj0(x: ArbFloat): ArbFloat;
const

       xvsmall = 3.2e-9;
          tbpi = 6.36619772367581343e-1;
           a1 : array[0..5] of ArbFloat =
           ( 1.22200000000000000e-17, -1.94383469000000000e-12,
             7.60816359241900000e-8,  -4.60626166206275050e-4,
             1.58067102332097261e-1,  -8.72344235285222129e-3);

            b1 : array[0..5] of ArbFloat =
            ( - 7.58850000000000000e-16, 7.84869631400000000e-11,
              - 1.76194690776215000e-6,  4.81918006946760450e-3,
              - 3.70094993872649779e-1,  1.57727971474890120e-1);

            c1 : array[0..4] of ArbFloat =
            ( 4.12532100000000000e-14, - 2.67925353056000000e-9,
              3.24603288210050800e-5,  - 3.48937694114088852e-2,
              2.65178613203336810e-1);

            d1 : array[0..13] of ArbFloat =
            ( 9.99457275788251954e-1, -5.36367319213004570e-4,
              6.13741608010926000e-6, -2.05274481565160000e-7,
              1.28037614434400000e-8, -1.21211819632000000e-9,
              1.55005642880000000e-10,-2.48827276800000000e-11,
              4.78702080000000000e-12,-1.06365696000000000e-12,
              2.45294080000000000e-13,-6.41843200000000000e-14,
              3.34028800000000000e-14,-1.17964800000000000e-14);

             d2 : array[0..16] of ArbFloat =
             ( -1.55551138795135187e-2,  6.83314909934390000e-5,
               -1.47713883264594000e-6,  7.10621485930000000e-8,
               -5.66871613024000000e-9,  6.43278173280000000e-10,
               -9.47034774400000000e-11, 1.70330918400000000e-11,
               -3.59094272000000000e-12, 8.59855360000000000e-13,
               -2.28807680000000000e-13, 6.95193600000000000e-14,
               -2.27942400000000000e-14, 4.75136000000000000e-15,
               -1.14688000000000000e-15, 2.12992000000000000e-15,
               -9.83040000000000000e-16);

var t, g, y, t2, a, b, c, cx, sx : ArbFloat;
    i, bov                       : ArbInt;

begin
  t:=abs(x);
  if t<=xvsmall
  then
    spebj0:=1
  else
  if t<=8
  then
    begin
      t:=0.03125*sqr(t)-1; t2:=2*t;
      b:=0; c:=0;
      bov:=sizeof(a1) div sizeof(ArbFloat) - 1;
      for i:=0 to bov do
        begin
          a:=t2*c-b+a1[i];
          if i<5
          then
            b:=t2*a-c+b1[i]
          else
            spebj0:=t*a-c+b1[i];
          if i<bov
          then
            c:=t2*b-a+c1[i]
          else
            if i<5
            then
              spebj0:=t*b-a+c1[i]
        end {i}
    end {abs(x)<=8}
  else
    begin
      g:=t-1/(2*tbpi); y:=sqrt(tbpi/t);
      cx:=cos(g)*y; sx:=-sin(g)*y*8/t;
      t:=128/sqr(t)-1;
      spebj0:=cx*spepol(t, d1[0], sizeof(d1) div sizeof(ArbFloat) - 1)
              + sx*spepol(t, d2[0], sizeof(d2) div sizeof(ArbFloat) - 1)
    end {abs(x)>8}

end {spebj0};

function spebj1(x: ArbFloat): ArbFloat;
const

       xvsmall = 3.2e-9;
          tbpi = 6.36619772367581343e-1;
      a1 : array[0..5] of ArbFloat =
      ( 2.95000000000000000e-18, -5.77740420000000000e-13,
        2.94970700727800000e-8,  -2.60444389348580680e-4,
        1.77709117239728283e-1,  -1.19180116054121687e+0);

      b1 : array[0..5] of ArbFloat =
      ( -1.95540000000000000e-16, 2.52812366400000000e-11,
        -7.61758780540030000e-7,  3.24027018268385747e-3,
        -6.61443934134543253e-1,  6.48358770605264921e-1);

      c1 : array[0..4] of ArbFloat =
      ( 1.13857200000000000e-14, -9.42421298160000000e-10,
        1.58870192399321300e-5,  -2.91755248061542077e-2,
        1.28799409885767762e+0);

       d1 : array[0..13] of ArbFloat =
       ( 1.00090702627808217e+0,  8.98804941670557880e-4,
        -7.95969469843846000e-6,  2.45367662227560000e-7,
        -1.47085129889600000e-8,  1.36030580128000000e-9,
        -1.71310758400000000e-10, 2.72040729600000000e-11,
        -5.19113984000000000e-12, 1.14622464000000000e-12,
        -2.63372800000000000e-13, 6.86387200000000000e-14,
        -3.54508800000000000e-14, 1.24928000000000000e-14);

       d2 : array[0..15] of ArbFloat =
       ( 4.67768740274489776e-2,  -9.62145882205441600e-5,
         1.82120185123076000e-6,  -8.29196070929200000e-8,
         6.42013250344000000e-9,  -7.15110504800000000e-10,
         1.03950931840000000e-10, -1.85248000000000000e-11,
         3.87554432000000000e-12, -9.23228160000000000e-13,
         2.50224640000000000e-13, -7.43936000000000000e-14,
         1.75718400000000000e-14, -4.83328000000000000e-15,
         5.32480000000000000e-15, -2.29376000000000000e-15);

var t, g, y, t2, a, b, c, cx, sx : ArbFloat;
    i, bov                       : ArbInt;

begin
  t:=abs(x);
  if t<xvsmall
  then
    spebj1:=x/2
  else
  if t<=8
  then
    begin
      t:=0.03125*sqr(t)-1; t2:=2*t;
      b:=0; c:=0;
      bov:=sizeof(a1) div sizeof(ArbFloat) - 1;
      for i:=0 to bov do
        begin
          a:=t2*c-b+a1[i];
          if i<bov
          then
            begin
              b:=t2*a-c+b1[i];
              c:=t2*b-a+c1[i]
            end
          else
            spebj1:=(x/8)*(t*a-c+b1[i])
        end {i}
    end {abs(x)<=8}
  else
    begin
      g:=t-1.5/tbpi; y:=sqrt(tbpi/t)*spesgn(x);
      cx:=cos(g)*y; sx:=-sin(g)*y*8/t;
      t:=128/sqr(t)-1;
      spebj1:=cx*spepol(t, d1[0], sizeof(d1) div sizeof(ArbFloat) - 1)
              + sx*spepol(t, d2[0], sizeof(d2) div sizeof(ArbFloat) - 1)
    end {abs(x)>8}
end {spebj1};

function spebk0(x: ArbFloat): ArbFloat;

const

     egam = 0.57721566490153286;
     xvsmall = 3.2e-9;
     highexp = 745;

      a0: array[0..7] of ArbFloat =
      ( 1.12896092945412762e+0,  1.32976966478338191e-1,
        4.07157485171389048e-3,  5.59702338227915383e-5,
        4.34562671546158210e-7,  2.16382411824721532e-9,
        7.49110736894134794e-12, 1.90674197514561280e-14);

      a1: array[0..8] of ArbFloat =
      ( 2.61841879258687055e-1,  1.52436921799395196e-1,
        6.63513979313943827e-3,  1.09534292632401542e-4,
        9.57878493265929443e-7,  5.19906865800665633e-9,
        1.92405264219706684e-11, 5.16867886946332160e-14,
        1.05407718191360000e-16);

      a2: array[0..22] of ArbFloat =
      ( 9.58210053294896496e-1, -1.42477910128828254e-1,
        3.23582010649653009e-2, -8.27780350351692662e-3,
        2.24709729617770471e-3, -6.32678357460594866e-4,
        1.82652460089342789e-4, -5.37101208898441760e-5,
        1.60185974149720562e-5, -4.83134250336922161e-6,
        1.47055796078231691e-6, -4.51017292375200017e-7,
        1.39217270224614153e-7, -4.32185089841834127e-8,
        1.34790467361340101e-8, -4.20597329258249948e-9,
        1.32069362385968867e-9, -4.33326665618780914e-10,
        1.37999268074442719e-10, -3.19241059198852137e-11,
        9.74410152270679245e-12, -7.83738609108569293e-12,
        2.57466288575820595e-12);

      a3: array[0..22] of ArbFloat =
     ( 6.97761598043851776e-1, -1.08801882084935132e-1,
       2.56253646031960321e-2, -6.74459607940169198e-3,
       1.87292939725962385e-3, -5.37145622971910027e-4,
       1.57451516235860573e-4, -4.68936653814896712e-5,
       1.41376509343622727e-5, -4.30373871727268511e-6,
       1.32052261058932425e-6, -4.07851207862189007e-7,
       1.26672629417567360e-7, -3.95403255713518420e-8,
       1.23923137898346852e-8, -3.88349705250555658e-9,
       1.22424982779432970e-9, -4.03424607871960089e-10,
       1.28905587479980147e-10,-2.97787564633235128e-11,
       9.11109430833001267e-12,-7.39672783987933184e-12,
       2.43538242247537459e-12);
      a4: array[0..16] of ArbFloat =
      ( 1.23688664769425422e+0,  -1.72683652385321641e-2,
       -9.25551464765637133e-4,  -9.02553345187404564e-5,
       -6.31692398333746470e-6,  -7.69177622529272933e-7,
       -4.16044811174114579e-8,  -9.41555321137176073e-9,
        1.75359321273580603e-10, -2.22829582288833265e-10,
        3.49564293256545992e-11, -1.11391758572647639e-11,
        2.85481235167705907e-12, -7.31344482663931904e-13,
        2.06328892562554880e-13, -1.28108310826991616e-13,
        4.43741979886551040e-14);


var t: ArbFloat;

begin
  if x<=0
  then
    RunError(401);
  if x<=xvsmall
  then
    spebk0:=-(ln(x/2)+egam)
  else
  if x<=1
  then
    begin
      t:=2*sqr(x)-1;
      spebk0:=-ln(x)*spepol(t, a0[0], sizeof(a0) div sizeof(ArbFloat) - 1)
              + spepol(t, a1[0], sizeof(a1) div sizeof(ArbFloat) - 1)
    end
  else
  if x<=2
  then
    spebk0:=exp(-x)*spepol(2*x-3, a2[0], sizeof(a2) div sizeof(ArbFloat) - 1)
  else
  if x<=4
  then
    spebk0:=exp(-x)*spepol(x-3, a3[0], sizeof(a3) div sizeof(ArbFloat) - 1)
  else
  if x <= highexp
  then
    spebk0:=exp(-x)*
            spepol(10/(1+x)-1, a4[0], sizeof(a4) div sizeof(ArbFloat) - 1)/sqrt(x)
  else
    spebk0:=0
end; {spebk0}

function spebk1(x: ArbFloat): ArbFloat;

const

   xsmall = 7.9e-10;
  highexp = 745;
   a0: array[0..7] of ArbFloat =
   ( 5.31907865913352762e-1,  3.25725988137110495e-2,
     6.71642805873498653e-4,  6.95300274548206237e-6,
     4.32764823642997753e-8,  1.79784792380155752e-10,
     5.33888268665658944e-13, 1.18964962439910400e-15);

   a1: array[0..7] of ArbFloat =
   ( 3.51825828289325536e-1,  4.50490442966943726e-2,
     1.20333585658219028e-3,  1.44612432533006139e-5,
     9.96686689273781531e-8,  4.46828628435618679e-10,
     1.40917103024514301e-12, 3.29881058019865600e-15);

   a2: array[0..23] of ArbFloat =
   ( 1.24316587355255299e+0, -2.71910714388689413e-1,
     8.20250220860693888e-2, -2.62545818729427417e-2,
     8.57388087067410089e-3, -2.82450787841655951e-3,
     9.34594154387642940e-4, -3.10007681013626626e-4,
     1.02982746700060730e-4, -3.42424912211942134e-5,
     1.13930169202553526e-5, -3.79227698821142908e-6,
     1.26265578331941923e-6, -4.20507152338934956e-7,
     1.40138351985185509e-7, -4.66928912168020101e-8,
     1.54456653909012693e-8, -5.13783508140332214e-9,
     1.82808381381205361e-9, -6.15211416898895086e-10,
     1.28044023949946257e-10, -4.02591066627023831e-11,
     4.27404330568767242e-11, -1.46639291782948454e-11);

   a3: array[0..23] of ArbFloat =
   ( 8.06563480128786903e-1,  -1.60052611291327173e-1,
     4.58591528414023064e-2,  -1.42363136684423646e-2,
     4.55865751206724687e-3,  -1.48185472032688523e-3,
     4.85707174778663652e-4,  -1.59994873621599146e-4,
     5.28712919123131781e-5,  -1.75089594354079944e-5,
     5.80692311842296724e-6,  -1.92794586996432593e-6,
     6.40581814037398274e-7,  -2.12969229346310343e-7,
     7.08723366696569880e-8,  -2.35855618461025265e-8,
     7.79421651144832709e-9,  -2.59039399308009059e-9,
     9.20781685906110546e-10, -3.09667392343245062e-10,
     6.44913423545894175e-11, -2.02680401514735862e-11,
     2.14736751065133220e-11, -7.36478297050421658e-12);

    a4: array[0..16] of ArbFloat =
    ( 1.30387573604230402e+0,   5.44845254318931612e-2,
      4.31639434283445364e-3,   4.29973970898766831e-4,
      4.04720631528495020e-5,   4.32776409784235211e-6,
      4.07563856931843484e-7,   4.86651420008153956e-8,
      3.82717692121438315e-9,   6.77688943857588882e-10,
      6.97075379117731379e-12,  1.72026097285930936e-11,
     -2.60774502020271104e-12,  8.58211523713560576e-13,
     -2.19287104441802752e-13,  1.39321122940600320e-13,
     -4.77850238111580160e-14);

var t: ArbFloat;

begin
  if x<=0
  then
    RunError(402);
  if x<=xsmall
  then
    spebk1:=1/x
  else
  if x<=1
  then
    begin
      t:=2*sqr(x)-1;
      spebk1:=( ln(x)*spepol(t, a0[0], sizeof(a0) div sizeof(ArbFloat) - 1)
              -spepol(t, a1[0], sizeof(a1) div sizeof(ArbFloat) -1) )*x + 1/x
    end
  else
  if x<=2
  then
    spebk1:=exp(-x)*spepol(2*x-3, a2[0], sizeof(a2) div sizeof(ArbFloat) - 1)
  else
  if x<=4
  then
    spebk1:=exp(-x)*spepol(x-3, a3[0], sizeof(a3) div sizeof(ArbFloat) - 1)
  else
  if x <= highexp
  then
    spebk1:=exp(-x)*spepol(10/(1+x)-1, a4[0],
            sizeof(a4) div sizeof(ArbFloat) - 1)/sqrt(x)
  else
    spebk1:=0
end; {spebk1}

function speby0(x: ArbFloat): ArbFloat;

const

      tbpi = 6.36619772367581343e-1;
      egam = 5.77215664901532861e-1;
   xvsmall = 3.2e-9;
   a1 : array[0..5] of ArbFloat =
   ( 3.90000000000000000e-19, -8.74734100000000000e-14,
     5.24879478733000000e-9,  -5.63207914105698700e-5,
     4.71966895957633869e-2,   1.79034314077182663e-1);

   b1 : array[0..5] of ArbFloat =
   ( -2.69800000000000000e-17, 4.02633082000000000e-12,
     -1.44072332740190000e-7,  7.53113593257774230e-4,
     -1.77302012781143582e-1, -2.74474305529745265e-1);

   c1 : array[0..5] of ArbFloat =
   ( 1.64349000000000000e-15, -1.58375525420000000e-10,
     3.20653253765480000e-6,  -7.28796247955207918e-3,
     2.61567346255046637e-1,  -3.31461132032849417e-2);

    d1 : array[0..13] of ArbFloat =
    ( 9.99457275788251954e-1, -5.36367319213004570e-4,
      6.13741608010926000e-6, -2.05274481565160000e-7,
      1.28037614434400000e-8, -1.21211819632000000e-9,
      1.55005642880000000e-10,-2.48827276800000000e-11,
      4.78702080000000000e-12,-1.06365696000000000e-12,
      2.45294080000000000e-13,-6.41843200000000000e-14,
      3.34028800000000000e-14,-1.17964800000000000e-14);

    d2 : array[0..16] of ArbFloat =
    (-1.55551138795135187e-2,  6.83314909934390000e-5,
     -1.47713883264594000e-6,  7.10621485930000000e-8,
     -5.66871613024000000e-9,  6.43278173280000000e-10,
     -9.47034774400000000e-11, 1.70330918400000000e-11,
     -3.59094272000000000e-12, 8.59855360000000000e-13,
     -2.28807680000000000e-13, 6.95193600000000000e-14,
     -2.27942400000000000e-14, 4.75136000000000000e-15,
     -1.14688000000000000e-15, 2.12992000000000000e-15,
     -9.83040000000000000e-16);

var t, g, y, t2, a, b, c, cx, sx : ArbFloat;
    i, bov                       : ArbInt;

begin
  if x<=0
  then
    RunError(403);
  if x<=xvsmall
  then
    speby0:=(ln(x/2)+egam)*tbpi
  else
  if x<=8
  then
    begin
      t:=0.03125*sqr(x)-1; t2:=2*t;
      b:=0; c:=0;
      bov:=sizeof(a1) div sizeof(ArbFloat) - 1;
      for i:=0 to bov do
        begin
          a:=t2*c-b+a1[i];
          b:=t2*a-c+b1[i];
          if i<bov
          then
            c:=t2*b-a+c1[i]
          else
            speby0:=t*b-a+c1[i]+tbpi*spebj0(x)*ln(x)
        end {i}
    end {x<=8}
  else
    begin
      g:=x-1/(2*tbpi); y:=sqrt(tbpi/x);
      cx:=cos(g)*y*8/x; sx:=sin(g)*y;
      t:=128/sqr(x)-1;
      speby0:=sx*spepol(t, d1[0], sizeof(d1) div sizeof(ArbFloat) - 1)
            + cx*spepol(t, d2[0], sizeof(d2) div sizeof(ArbFloat) - 1)
    end {x>8}
end {speby0};

function speby1(x: ArbFloat): ArbFloat;

const
    tbpi = 6.36619772367581343e-1;
    xsmall = 7.9e-10;
   a1 : array[0..5] of ArbFloat =
   (-6.58000000000000000e-18, 1.21143321000000000e-12,
    -5.68844003991900000e-8,  4.40478629867099510e-4,
    -2.26624991556754924e-1, -1.28697384381350001e-1);

   b1 : array[0..5] of ArbFloat =
   ( 4.27730000000000000e-16,-5.17212147300000000e-11,
     1.41662436449235000e-6, -5.13164116106108479e-3,
     6.75615780772187667e-1,  2.03041058859342538e-2);

   c1 : array[0..4] of ArbFloat =
   (-2.44094900000000000e-14, 1.87547032473000000e-9,
    -2.83046401495148000e-5,  4.23191803533369041e-2,
    -7.67296362886645940e-1);

   d1 : array[0..13] of ArbFloat =
   ( 1.00090702627808217e+0,  8.98804941670557880e-4,
    -7.95969469843846000e-6,  2.45367662227560000e-7,
    -1.47085129889600000e-8,  1.36030580128000000e-9,
    -1.71310758400000000e-10, 2.72040729600000000e-11,
    -5.19113984000000000e-12, 1.14622464000000000e-12,
    -2.63372800000000000e-13, 6.86387200000000000e-14,
    -3.54508800000000000e-14, 1.24928000000000000e-14);

    d2 : array[0..15] of ArbFloat =
    ( 4.67768740274489776e-2, -9.62145882205441600e-5,
      1.82120185123076000e-6, -8.29196070929200000e-8,
      6.42013250344000000e-9, -7.15110504800000000e-10,
      1.03950931840000000e-10,-1.85248000000000000e-11,
      3.87554432000000000e-12,-9.23228160000000000e-13,
      2.50224640000000000e-13,-7.43936000000000000e-14,
      1.75718400000000000e-14,-4.83328000000000000e-15,
      5.32480000000000000e-15,-2.29376000000000000e-15);

var t, g, y, t2, a, b, c, cx, sx : ArbFloat;
    i, bov                       : ArbInt;

begin
  if x<=0
  then
    RunError(404);
  if x<=xsmall
  then
    speby1:=-tbpi/x
  else
  if x<=8
  then
    begin
      t:=0.03125*sqr(x)-1; t2:=2*t;
      b:=0; c:=0;
      bov:=sizeof(a1) div sizeof(ArbFloat) - 1;
      for i:=0 to bov do
        begin
          a:=t2*c-b+a1[i];
          if i<bov
          then
            begin
              b:=t2*a-c+b1[i];
              c:=t2*b-a+c1[i]
            end
          else
          if bov=3   {single}
          then
            begin
              b:=t2*a-c+b1[i];
              speby1:=(t*b-a+c1[i])*x/8 + spebj1(x)*ln(x)*tbpi - tbpi/x
            end
          else
            speby1:=(t*a-c+b1[i])*x/8 + spebj1(x)*ln(x)*tbpi - tbpi/x
        end {i}
    end {x<=8}
  else
    begin
      g:=x-3/(2*tbpi); y:=sqrt(tbpi/x);
      cx:=cos(g)*y*8/x; sx:=sin(g)*y;
      t:=128/sqr(x)-1;
      speby1:=sx*spepol(t, d1[0], sizeof(d1) div sizeof(ArbFloat) - 1)
            + cx*spepol(t, d2[0], sizeof(d2) div sizeof(ArbFloat) - 1)
    end {x>8}
end {speby1};

function speent(x : ArbFloat): longint;

var tx : longint;

begin
  tx:=trunc(x);
  if x>=0
  then
    speent:=tx
  else
    if x=tx
    then
      speent:=tx
    else
      speent:=tx-1
end; {speent}

function speerf(x : ArbFloat) : ArbFloat;
const

        xup = 6.25;
     sqrtpi = 1.7724538509055160;
     c : array[1..18] of ArbFloat =
     ( 1.9449071068178803e0,  4.20186582324414e-2, -1.86866103976769e-2,
       5.1281061839107e-3,   -1.0683107461726e-3,   1.744737872522e-4,
      -2.15642065714e-5,      1.7282657974e-6,     -2.00479241e-8,
      -1.64782105e-8,         2.0008475e-9,         2.57716e-11,
      -3.06343e-11,           1.9158e-12,           3.703e-13,
      -5.43e-14,             -4.0e-15,              1.2e-15);

     d : array[1..17] of ArbFloat =
     ( 1.4831105640848036e0, -3.010710733865950e-1, 6.89948306898316e-2,
      -1.39162712647222e-2,   2.4207995224335e-3,  -3.658639685849e-4,
       4.86209844323e-5,     -5.7492565580e-6,      6.113243578e-7,
      -5.89910153e-8,         5.2070091e-9,        -4.232976e-10,
       3.18811e-11,          -2.2361e-12,           1.467e-13,
      -9.0e-15,               5.0e-16);

  var t, s, s1, s2, x2: ArbFloat;
         bovc, bovd, j: ArbInt;
begin
  bovc:=sizeof(c) div sizeof(ArbFloat);
  bovd:=sizeof(d) div sizeof(ArbFloat);
  t:=abs(x);
  if t <= 2
  then
    begin
      x2:=sqr(x)-2;
      s1:=d[bovd]; s2:=0; j:=bovd-1;
      s:=x2*s1-s2+d[j];
      while j > 1 do
        begin
          s2:=s1; s1:=s; j:=j-1;
          s:=x2*s1-s2+d[j]
        end;
      speerf:=(s-s2)*x/2
    end
  else
    if t < xup
    then
      begin
        x2:=2-20/(t+3);
        s1:=c[bovc]; s2:=0; j:=bovc-1;
        s:=x2*s1-s2+c[j];
        while j > 1 do
          begin
            s2:=s1; s1:=s; j:=j-1;
            s:=x2*s1-s2+c[j]
          end;
        x2:=((s-s2)/(2*t))*exp(-sqr(x))/sqrtpi;
        speerf:=(1-x2)*spesgn(x)
      end
    else
      speerf:=spesgn(x)
end;  {speerf}

function spemax(a, b: ArbFloat): ArbFloat;
begin
  if a>b
  then
    spemax:=a
  else
    spemax:=b
end; {spemax}

function speefc(x : ArbFloat) : ArbFloat;
const

   xlow = -6.25;
  xhigh = 27.28;
      c : array[0..22] of ArbFloat =
      ( 1.455897212750385e-1, -2.734219314954260e-1,
        2.260080669166197e-1, -1.635718955239687e-1,
        1.026043120322792e-1, -5.480232669380236e-2,
        2.414322397093253e-2, -8.220621168415435e-3,
        1.802962431316418e-3, -2.553523453642242e-5,
       -1.524627476123466e-4,  4.789838226695987e-5,
        3.584014089915968e-6, -6.182369348098529e-6,
        7.478317101785790e-7,  6.575825478226343e-7,
       -1.822565715362025e-7, -7.043998994397452e-8,
        3.026547320064576e-8,  7.532536116142436e-9,
       -4.066088879757269e-9, -5.718639670776992e-10,
        3.328130055126039e-10);

  var t, s : ArbFloat;
begin
  if x <= xlow
  then
    speefc:=2
  else
  if x >= xhigh
  then
    speefc:=0
  else
    begin
      t:=1-7.5/(abs(x)+3.75);
      s:=exp(-x*x)*spepol(t, c[0], sizeof(c) div sizeof(ArbFloat) - 1);
      if x < 0
      then
        speefc:=2-s
      else
        speefc:=s
    end
end {speefc};

function spegam(x: ArbFloat): ArbFloat;
const

    tmax = 170;
    a: array[0..23] of ArbFloat =
    ( 8.86226925452758013e-1,  1.61691987244425092e-2,
      1.03703363422075456e-1, -1.34118505705967765e-2,
      9.04033494028101968e-3, -2.42259538436268176e-3,
      9.15785997288933120e-4, -2.96890121633200000e-4,
      1.00928148823365120e-4, -3.36375833240268800e-5,
      1.12524642975590400e-5, -3.75499034136576000e-6,
      1.25281466396672000e-6, -4.17808776355840000e-7,
      1.39383522590720000e-7, -4.64774927155200000e-8,
      1.53835215257600000e-8, -5.11961333760000000e-9,
      1.82243164160000000e-9, -6.13513953280000000e-10,
      1.27679856640000000e-10,-4.01499750400000000e-11,
      4.26560716800000000e-11,-1.46381209600000000e-11);

var tvsmall, t, g: ArbFloat;
             m, i: ArbInt;
begin
  if sizeof(ArbFloat) = 6
  then
    tvsmall:=2*midget
  else
    tvsmall:=midget;
  t:=abs(x);
  if t > tmax
  then
    RunError(407);
  if t < macheps
  then
    begin
      if t < tvsmall
      then
        RunError(407);
      spegam:=1/x
    end
  else  { abs(x) >= macheps }
    begin
      m:=trunc(x);
      if x > 0
      then
        begin
          t:=x-m; m:=m-1; g:=1;
          if m<0
          then
            g:=g/x
          else
            if m>0
            then
              for i:=1 to m do
                g:=(x-i)*g
        end
      else { x < 0 }
        begin
          t:=x-m+1;
          if t=1
          then
            RunError(407);
          m:=1-m;
          g:=x;
          for i:=1 to m do
            g:=(i+x)*g;
          g:=1/g
        end;
      spegam:=spepol(2*t-1, a[0], sizeof(a) div sizeof(ArbFloat) - 1)*g
    end { abs(x) >= macheps }
end; {spegam}

function spelga(x: ArbFloat): ArbFloat;

const

    xbig = 7.7e7;
    xmax = 2.559e305;
  lnr2pi = 9.18938533204672742e-1;
    a: array[0..23] of ArbFloat =
    ( 8.86226925452758013e-1,  1.61691987244425092e-2,
      1.03703363422075456e-1, -1.34118505705967765e-2,
      9.04033494028101968e-3, -2.42259538436268176e-3,
      9.15785997288933120e-4, -2.96890121633200000e-4,
      1.00928148823365120e-4, -3.36375833240268800e-5,
      1.12524642975590400e-5, -3.75499034136576000e-6,
      1.25281466396672000e-6, -4.17808776355840000e-7,
      1.39383522590720000e-7, -4.64774927155200000e-8,
      1.53835215257600000e-8, -5.11961333760000000e-9,
      1.82243164160000000e-9, -6.13513953280000000e-10,
      1.27679856640000000e-10,-4.01499750400000000e-11,
      4.26560716800000000e-11,-1.46381209600000000e-11);
    b: array[0..5] of ArbFloat =
    ( 8.33271644065786580e-2,  -6.16502049453716986e-6,
      3.89978899876484712e-9,  -6.45101975779653651e-12,
      2.00201927337982364e-14, -9.94561064728159347e-17);


var t, g : ArbFloat;
    m, i : ArbInt;

begin
  if x <= 0
  then
    RunError(408);
  if x <= macheps
  then
    spelga:=-ln(x)
  else
  if x <= 15
  then
    begin
      m:=trunc(x); t:=x-m; m:=m-1; g:=1;
      if m < 0
      then
        g:=g/x
      else
      if m > 0
      then
        for i:=1 to m do
          g:=(x-i)*g;
      spelga:=ln(g*spepol(2*t-1, a[0], sizeof(a) div sizeof(ArbFloat) - 1))
    end
  else { x > 15 }
  if x <= xbig
  then
    spelga:=(x-0.5)*ln(x) - x + lnr2pi
            + spepol(450/sqr(x)-1, b[0], sizeof(b) div sizeof(ArbFloat) - 1)/x
  else { x > xbig }
  if x <= xmax
  then
    spelga:=(x-0.5)*ln(x) - x + lnr2pi
  else  { x > xmax => x*ln(x) > giant }
    RunError(408)
end; {spelga}

function spepol(x: ArbFloat; var a: ArbFloat; n: ArbInt): ArbFloat;
var   pa : ^arfloat0;
       i : ArbInt;
    polx : ArbFloat;
begin
  pa:=@a;
  polx:=0;
  for i:=n downto 0 do
    polx:=polx*x+pa^[i];
  spepol:=polx
end {spepol};

function spepow(a, b: ArbFloat): ArbFloat;

   function PowInt(a: double; n: longint): double;
   var a1 : double;
   begin
     if n=0 then PowInt := 1 else
     begin
        a1 := 1;
        if n<0 then begin a := 1/a; n := -n end;
        while n>0
        do if Odd(n)
           then begin Dec(n); a1 := a1*a end
           else begin n := n div 2; a := sqr(a) end;
        PowInt := a1
     end
   end;

var tb : longint;
    fb : double;
begin

  { (a < 0, b niet geheel) of (a = 0, b <= 0), dan afbreken}
  if (a=0) then if (b<=0) then RunError(400) else begin SpePow :=0; exit end;
  tb := Trunc(b); fb := b-tb;
  if (a<0) and (fb<>0) then RunError(400);

  if a>0
  then if fb=0 then SpePow := PowInt(a, tb)
               else SpePow := PowInt(a, tb)*exp(fb*ln(a))
  else if odd(tb) then Spepow := -PowInt(-a, tb)
                  else SpePow := PowInt(-a, tb)

end; {spepow}

function spesgn(x: ArbFloat): ArbInt;

begin
  if x<0
  then
    spesgn:=-1
  else
    if x=0
    then
      spesgn:=0
    else
      spesgn:=1
end; {spesgn}

function spears(x: ArbFloat): ArbFloat;
const

    pi2 = 1.570796326794897;
    a : array[0..17] of ArbFloat =
    (  1.047197551196598e+0, 5.375149359132719e-2, 7.798902238957732e-3,
       1.519668539582420e-3, 3.408637238430600e-4, 8.302317819598986e-5,
       2.134554822576075e-5, 5.701781046148566e-6, 1.566985123962741e-6,
       4.402076871418002e-7, 1.257811162594110e-7, 3.646577948300129e-8,
       1.081021746966715e-8, 3.212744286269388e-9, 8.515014302985799e-10,
       2.513296398553196e-10, 1.342121568282535e-10, 4.210346761190271e-11);

var    y, u, t, s : ArbFloat;
    uprang        : boolean;
begin
  if abs(x) > 1
  then
    RunError(401);
  u:=sqr(x); uprang:= u > 0.5;
  if uprang
  then
    u:=1-u;
  t:=4*u-1; y:=spepol(t, a[0], sizeof(a) div sizeof(ArbFloat) - 1);
  if uprang
  then
    begin
      s:=pi2-sqrt(u)*y;
      if x < 0
      then
        s:=-s;
      spears:=s
    end
  else
    spears:=x*y
end;  {spears}

function spearc(x: ArbFloat): ArbFloat;
const

    pi2 = 1.570796326794897;
    a : array[0..17] of ArbFloat =
    ( 1.047197551196598e+0,  5.375149359132719e-2,  7.798902238957732e-3,
      1.519668539582420e-3,  3.408637238430600e-4,  8.302317819598986e-5,
      2.134554822576075e-5,  5.701781046148566e-6,  1.566985123962741e-6,
      4.402076871418002e-7,  1.257811162594110e-7,  3.646577948300129e-8,
      1.081021746966715e-8,  3.212744286269388e-9,  8.515014302985799e-10,
      2.513296398553196e-10, 1.342121568282535e-10, 4.210346761190271e-11);

var u, t, y, s    : ArbFloat;
    uprang        : boolean;
begin
  if abs(x)>1.0
  then
    RunError(402);
  u:=sqr(x); uprang:=u>0.5;
  if uprang
  then
    u:=1-u;
  t:=4*u-1; y:=spepol(t, a[0], sizeof(a) div sizeof(ArbFloat) - 1);
  if uprang
  then
    begin
      s:=sqrt(u)*y;
      if x<0
      then
        s:=2*pi2-s;
      spearc:=s
    end
  else
    spearc:=pi2-x*y
end;  {spearc}

function spesih(x: ArbFloat): ArbFloat;
const

    a : array[0..6] of ArbFloat =
    ( 1.085441641272607e+0,  8.757509762437522e-2,  2.158779361257021e-3,
      2.549839945498292e-5,  1.761854853281383e-7,  7.980704288665359e-10,
      2.551377137317034e-12);

var u : ArbFloat;
begin
  if abs(x)<=1.0
  then
    begin
      u:=2*sqr(x)-1;
      spesih:=x*spepol(u, a[0], sizeof(a) div sizeof(ArbFloat) - 1)
    end
  else
  begin
    u:=exp(x); spesih:=(u-1/u)/2
  end
end; {spesih}

function specoh(x: ArbFloat): ArbFloat;
var u: ArbFloat;
begin
  u:=exp(x); specoh:=(u+1/u)/2
end; {specoh}

function spetah(x: ArbFloat): ArbFloat;
const
    xhi = 18.50;
    a : array[0..15] of ArbFloat =
    ( 8.610571715805476e-1, -1.158834489728470e-1,  1.918072383973950e-2,
     -3.225255180728459e-3,  5.433071386922689e-4, -9.154289983175165e-5,
      1.542469328074432e-5, -2.599022539340038e-6,  4.379282308765732e-7,
     -7.378980192173815e-8,  1.243517352745986e-8, -2.095373768837420e-9,
      3.509758916273561e-10,-5.908745181531817e-11, 1.124199312776748e-11,
     -1.907888434471600e-12);

var t, y: ArbFloat;

begin
  t:=abs(x);
  if t <= 1
  then
    begin
      y:=2*sqr(x)-1;
      spetah:=x*spepol(y, a[0], sizeof(a) div sizeof(ArbFloat) - 1)
    end
  else
  if t < xhi
  then
    begin
      y:=exp(2*x); spetah:=(y-1)/(y+1)
    end
  else
    spetah:=spesgn(x)
end; {spetah}

function speash(x: ArbFloat): ArbFloat;
const

    xhi = 1e9;
    c : array[0..18] of ArbFloat =
    (  9.312298594527122e-1,  -5.736663926249348e-2,
       9.004288574881897e-3,  -1.833458667045431e-3,
       4.230023450529706e-4,  -1.050715136470630e-4,
       2.740790473603819e-5,  -7.402952157663977e-6,
       2.052474396638805e-6,  -5.807433412373489e-7,
       1.670117348345774e-7,  -4.863477336087045e-8,
       1.432753532351304e-8,  -4.319978113584910e-9,
       1.299779213740398e-9,  -3.394726871170490e-10,
       1.008344962167889e-10, -5.731943029121004e-11,
       1.810792296549804e-11);


var t : ArbFloat;

begin
  t:=abs(x);
  if t <= 1 then
    speash:=x*spepol(2*sqr(x)-1, c[0], sizeof(c) div sizeof(ArbFloat) - 1)
  else
  if t < xhi then
    speash:=ln(sqrt(sqr(x)+1)+t)*spesgn(x)
  else
    speash:=ln(2*t)*spesgn(x)
end; {speash}

function speach(x: ArbFloat): ArbFloat;
const

    xhi = 1e9;

begin
  if x<1 then
    RunError(405);
  if x=1 then
    speach:=0
  else
  if x<=xhi then
    speach:=ln(x+sqrt(sqr(x)-1))
  else
    speach:=ln(2*x)
end; {speach}

function speath(x: ArbFloat): ArbFloat;
const

    c : array[0..19] of ArbFloat =
    ( 1.098612288668110e+0,  1.173605223326117e-1,  2.309071936165689e-2,
      5.449091889986991e-3,  1.404884102286929e-3,  3.816948426588058e-4,
      1.073604335435426e-4,  3.095027782918129e-5,  9.088050814470148e-6,
      2.706881064641104e-6,  8.155200644023077e-7,  2.479830612463254e-7,
      7.588067811453948e-8,  2.339295963220429e-8,  7.408486568719348e-9,
      2.319454882064018e-9,  5.960921368486746e-10, 1.820410351379402e-10,
      1.184977617320312e-10, 3.856235316559190e-11);

var t, u: ArbFloat;
begin
  t:=abs(x);
  if t >= 1 then
    RunError(406);
  u:=sqr(x);
  if u < 0.5 then
    speath:=x*spepol(4*u-1, c[0], sizeof(c) div sizeof(ArbFloat) - 1)
  else { 0.5 < x*x < 1 }
    speath:=ln((1+t)/(1-t))/2*spesgn(x)
end; {speath}

var exitsave : pointer;

procedure MyExit;
const ErrorS : array[400..408,1..6] of char =
     ('spepow',
      'spebk0',
      'spebk1',
      'speby0',
      'speby1',
      'speach',
      'speath',
      'spegam',
      'spelga');

<<<<<<< HEAD
<<<<<<< HEAD
//var ErrFil : text;

begin
     ExitProc := ExitSave;
//     Assign(ErrFil, 'CON');
//    ReWrite(ErrFil);
     if (ExitCode>=400) AND (ExitCode<=408) then
       begin
//         write(ErrFil, 'critical error in ', ErrorS[ExitCode]);
         ExitCode := 201
       end;
//     Close(ErrFil)
=======
=======
>>>>>>> origin/fixes_2_2
var ErrFil : text;

begin
     ExitProc := ExitSave;
     Assign(ErrFil, 'CON');
     ReWrite(ErrFil);
     if (ExitCode>=400) AND (ExitCode<=408) then
       begin
         write(ErrFil, 'critical error in ', ErrorS[ExitCode]);
         ExitCode := 201
       end;
     Close(ErrFil)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

begin
   ExitSave := ExitProc;
   ExitProc := @MyExit;
end.
