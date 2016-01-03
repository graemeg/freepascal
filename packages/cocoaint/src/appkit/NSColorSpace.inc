{ Parsed from Appkit.framework NSColorSpace.h }

{$ifdef TYPES}
{$ifndef NSCOLORSPACE_PAS_T}
{$define NSCOLORSPACE_PAS_T}

{ Constants }

const
  NSUnknownColorSpaceModel = -1;
  NSGrayColorSpaceModel = 0;
  NSRGBColorSpaceModel = 1;
  NSCMYKColorSpaceModel = 2;
  NSLABColorSpaceModel = 3;
  NSDeviceNColorSpaceModel = 4;
  NSIndexedColorSpaceModel = 5;
  NSPatternColorSpaceModel = 6;

{ Types }
type
  NSColorSpaceModel = NSInteger;
  NSColorSpaceModelPtr = ^NSColorSpaceModel;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCOLORSPACE_PAS_R}
{$define NSCOLORSPACE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCOLORSPACE_PAS_F}
{$define NSCOLORSPACE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCOLORSPACE_PAS_S}
{$define NSCOLORSPACE_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSColorSpace = objcclass;
  NSColorSpacePointer = ^NSColorSpace;
  NSColorSpacePtr = NSColorSpacePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCOLORSPACE_PAS_C}
{$define NSCOLORSPACE_PAS_C}

{ NSColorSpace }
  NSColorSpace = objcclass external (NSObject, NSCodingProtocol)
  private
    _profile: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSColorSpace2: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSColorSpace2: cuint);
       1: (
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
        colorSpaceID: 0..((1 shl 8)-1);
        storageType: 0..((1 shl 3)-1);
        replacedDuringUnarchiving: 0..1;
        _anonbitfield_NSColorSpace0: 0..((1 shl 20)-1);
        {$ifdef cpu64}
        _anonbitfield_NSColorSpace1: 0..((1 shl 32)-1);
        {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
      );
      end;
    __cgColorSpace: Pointer;
    _reserved: array[0..(5)-1] of Pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    __cgColorSpace: Pointer;
        _reserved: array[0..4] of Pointer;
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
    
  public
    function initWithICCProfileData(iccData: NSData): id; message 'initWithICCProfileData:';
    function ICCProfileData: NSData; message 'ICCProfileData';
    function initWithColorSyncProfile(prof: Pointer): id; message 'initWithColorSyncProfile:';
    function colorSyncProfile: Pointer; message 'colorSyncProfile';
    function initWithCGColorSpace(cgColorSpace_: CGColorSpaceRef): id; message 'initWithCGColorSpace:';
    function CGColorSpace: CGColorSpaceRef; message 'CGColorSpace';
    function numberOfColorComponents: NSInteger; message 'numberOfColorComponents';
    function colorSpaceModel: NSColorSpaceModel; message 'colorSpaceModel';
    function localizedName: NSString; message 'localizedName';
    class function genericRGBColorSpace: NSColorSpace; message 'genericRGBColorSpace';
    class function genericGrayColorSpace: NSColorSpace; message 'genericGrayColorSpace';
    class function genericCMYKColorSpace: NSColorSpace; message 'genericCMYKColorSpace';
    class function deviceRGBColorSpace: NSColorSpace; message 'deviceRGBColorSpace';
    class function deviceGrayColorSpace: NSColorSpace; message 'deviceGrayColorSpace';
    class function deviceCMYKColorSpace: NSColorSpace; message 'deviceCMYKColorSpace';
    class function sRGBColorSpace: NSColorSpace; message 'sRGBColorSpace';
    class function genericGamma22GrayColorSpace: NSColorSpace; message 'genericGamma22GrayColorSpace';
    class function adobeRGB1998ColorSpace: NSColorSpace; message 'adobeRGB1998ColorSpace';
    class function availableColorSpacesWithModel(model: NSColorSpaceModel): NSArray; message 'availableColorSpacesWithModel:';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
