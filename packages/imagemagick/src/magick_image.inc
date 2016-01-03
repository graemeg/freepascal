{
  Copyright 1999-2005 ImageMagick Studio LLC, a non-profit organization
  dedicated to making software imaging solutions freely available.
  
  You may not use this file except in compliance with the License.
  obtain a copy of the License at
  
    http://www.imagemagick.org/script/license.php
  
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  Magick Wand Image Methods.
}

{ BugFix :
  - MagickDrawImage(...const drawing_wand: PMagickWand...
  + MagickDrawImage(...const drawing_wand: PDrawingWand...
  - MagickAffineTransformImage(...const drawing_wand: PMagickWand...
  + MagickAffineTransformImage(...const drawing_wand: PDrawingWand...
  - MagickAnnotateImage(...const drawing_wand: PMagickWand...
  + MagickAnnotateImage(...const drawing_wand: PDrawingWand...
}

function MagickGetImageChannelStatistics(wand: PMagickWand): PChannelStatistics; cdecl; external WandExport;

function MagickGetImageAttribute(wand: PMagickWand): PChar; cdecl; external WandExport;
function MagickGetImageFilename(wand: PMagickWand): PChar; cdecl; external WandExport;
function MagickGetImageFormat(wand: PMagickWand): PChar; cdecl; external WandExport;
function MagickGetImageSignature(wand: PMagickWand): PChar; cdecl; external WandExport;
function MagickIdentifyImage(wand: PMagickWand): PChar; cdecl; external WandExport;

function MagickGetImageCompose(wand: PMagickWand): CompositeOperator; cdecl; external WandExport;

function MagickGetImageColorspace(wand: PMagickWand): ColorspaceType; cdecl; external WandExport;

function MagickGetImageCompression(wand: PMagickWand): CompressionType; cdecl; external WandExport;

function MagickGetImageDispose(wand: PMagickWand): DisposeType; cdecl; external WandExport;

function MagickGetImageGamma(wand: PMagickWand): double; cdecl; external WandExport;
function MagickGetImageTotalInkDensity(wand: PMagickWand): double; cdecl; external WandExport;

function GetImageFromMagickWand(wand: PMagickWand): PImage; cdecl; external WandExport;

function MagickGetImageType(wand: PMagickWand): ImageType; cdecl; external WandExport;

function MagickGetImageInterlaceScheme(wand: PMagickWand): InterlaceType; cdecl; external WandExport;

<<<<<<< HEAD
<<<<<<< HEAD
function MagickGetImageIndex(wand: PMagickWand): clong; cdecl; external WandExport;

function MagickAdaptiveThresholdImage(wand: PMagickWand;
     const width, height: culong; const offset: clong): MagickBooleanType; cdecl; external WandExport;
=======
=======
>>>>>>> origin/fixes_2_2
function MagickGetImageIndex(wand: PMagickWand): Integer; cdecl; external WandExport;

function MagickAdaptiveThresholdImage(wand: PMagickWand;
     const width, height: Cardinal; const offset: Integer): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MagickAddImage(wand: PMagickWand; const add_wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickAddNoiseImage(wand: PMagickWand; const noise_type: NoiseType): MagickBooleanType; cdecl; external WandExport;
function MagickAffineTransformImage(wand: PMagickWand;
 const drawing_wand: PDrawingWand): MagickBooleanType; cdecl; external WandExport;
function MagickAnnotateImage(wand: PMagickWand;
 const drawing_wand: PDrawingWand; const x, y, angle: double;
 const text: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickAnimateImages(wand: PMagickWand;
 const server_name: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickBlackThresholdImage(wand: PMagickWand;
 const threshold: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickBlurImage(wand: PMagickWand;
 const radius, sigma: double): MagickBooleanType; cdecl; external WandExport;
function MagickBlurImageChannel(wand: PMagickWand;
 const channel: ChannelType; const radius, sigma: double): MagickBooleanType; cdecl; external WandExport;
function MagickBorderImage(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const bordercolor: PPixelWand; const width, height: culong): MagickBooleanType; cdecl; external WandExport;
function MagickCharcoalImage(wand: PMagickWand;
   const radius, sigma: double): MagickBooleanType; cdecl; external WandExport;
function MagickChopImage(wand: PMagickWand;
 const width, height: culong; const x, y: clong): MagickBooleanType; cdecl; external WandExport;
=======
=======
>>>>>>> origin/fixes_2_2
 const bordercolor: PPixelWand; const width, height: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickCharcoalImage(wand: PMagickWand;
   const radius, sigma: double): MagickBooleanType; cdecl; external WandExport;
function MagickChopImage(wand: PMagickWand;
 const width, height: Cardinal; const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MagickClipImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickClipPathImage(wand: PMagickWand;
 const pathname: PChar; const inside: MagickBooleanType): MagickBooleanType; cdecl; external WandExport;
function MagickColorFloodfillImage(wand: PMagickWand;
 const fill: PPixelWand; const fuzz: double; const bordercolor: PPixelWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const x, y: clong): MagickBooleanType; cdecl; external WandExport;
=======
 const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
>>>>>>> graemeg/fixes_2_2
=======
 const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
>>>>>>> origin/fixes_2_2
function MagickColorizeImage(wand: PMagickWand;
 const colorize, opacity: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickCommentImage(wand: PMagickWand;
 const comment: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickCompositeImage(wand: PMagickWand;
 const composite_wand: PMagickWand; const compose: CompositeOperator;
<<<<<<< HEAD
<<<<<<< HEAD
 const x, y: clong): MagickBooleanType; cdecl; external WandExport;
function MagickConstituteImage(wand: PMagickWand;
 const columns, rows: culong; const map: PChar;
=======
 const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
function MagickConstituteImage(wand: PMagickWand;
 const columns, rows: Cardinal; const map: PChar;
>>>>>>> graemeg/fixes_2_2
=======
 const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
function MagickConstituteImage(wand: PMagickWand;
 const columns, rows: Cardinal; const map: PChar;
>>>>>>> origin/fixes_2_2
 const storage: StorageType; pixels: Pointer): MagickBooleanType; cdecl; external WandExport;
function MagickContrastImage(wand: PMagickWand;
 const sharpen: MagickBooleanType): MagickBooleanType; cdecl; external WandExport;
function MagickConvolveImage(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const order: culong; const kernel: PDouble): MagickBooleanType; cdecl; external WandExport;
function MagickConvolveImageChannel(wand: PMagickWand;
 const channel: ChannelType; const order: culong;
 const kernel: PDouble): MagickBooleanType; cdecl; external WandExport;
function MagickCropImage(wand: PMagickWand;
 const width, height: culong; const x, y: clong): MagickBooleanType; cdecl; external WandExport;
function MagickCycleColormapImage(wand: PMagickWand;
 const displace: clong): MagickBooleanType; cdecl; external WandExport;
=======
=======
>>>>>>> origin/fixes_2_2
 const order: Cardinal; const kernel: PDouble): MagickBooleanType; cdecl; external WandExport;
function MagickConvolveImageChannel(wand: PMagickWand;
 const channel: ChannelType; const order: Cardinal;
 const kernel: PDouble): MagickBooleanType; cdecl; external WandExport;
function MagickCropImage(wand: PMagickWand;
 const width, height: Cardinal; const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
function MagickCycleColormapImage(wand: PMagickWand;
 const displace: Integer): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MagickDespeckleImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickDisplayImage(wand: PMagickWand;
 const server_name: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickDisplayImages(wand: PMagickWand;
 const server_name: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickDrawImage(wand: PMagickWand;
 const drawing_wand: PDrawingWand): MagickBooleanType; cdecl; external WandExport;
function MagickEdgeImage(wand: PMagickWand;
 const radius: double): MagickBooleanType; cdecl; external WandExport;
function MagickEmbossImage(wand: PMagickWand;
 const radius, sigma: double): MagickBooleanType; cdecl; external WandExport;
function MagickEnhanceImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickEqualizeImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickEvaluateImage(wand: PMagickWand;
 const op: MagickEvaluateOperator; const constant: double): MagickBooleanType; cdecl; external WandExport;
function MagickEvaluateImageChannel(wand: PMagickWand;
 const op: MagickEvaluateOperator; const constant: double): MagickBooleanType; cdecl; external WandExport;
function MagickFlipImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickFlopImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickFrameImage(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const matte_color: PPixelWand; const width, height: culong;
 const inner_bevel, outer_bevel: clong): MagickBooleanType; cdecl; external WandExport;
=======
 const matte_color: PPixelWand; const width, height: Cardinal;
 const inner_bevel, outer_bevel: Integer): MagickBooleanType; cdecl; external WandExport;
>>>>>>> graemeg/fixes_2_2
=======
 const matte_color: PPixelWand; const width, height: Cardinal;
 const inner_bevel, outer_bevel: Integer): MagickBooleanType; cdecl; external WandExport;
>>>>>>> origin/fixes_2_2
function MagickGammaImage(wand: PMagickWand; const gamma: double): MagickBooleanType; cdecl; external WandExport;
function MagickGammaImageChannel(wand: PMagickWand;
 const channel: ChannelType; const gamma: double): MagickBooleanType; cdecl; external WandExport;
function MagickGaussianBlurImage(wand: PMagickWand;
 const radius, sigma: double): MagickBooleanType; cdecl; external WandExport;
function MagickGaussianBlurImageChannel(wand: PMagickWand;
 const channel: ChannelType; const radius, sigma: double): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageBackgroundColor(wand: PMagickWand;
 background_color: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageBluePrimary(wand: PMagickWand; x, y: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageBorderColor(wand: PMagickWand; border_color: PPixelWand
 ): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageChannelDistortion(wand: PMagickWand;
 const reference: PMagickWand; const channel: ChannelType;
 const metric: MetricType; distortion: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageDistortion(wand: PMagickWand;
 const reference: PMagickWand; const metric: MetricType;
 distortion: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageChannelExtrema(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const channel: ChannelType; minima, maxima: Pculong): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageChannelMean(wand: PMagickWand;
 const channel: ChannelType; mean, standard_deviation: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageColormapColor(wand: PMagickWand;
 const index: culong; color: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageExtrema(wand: PMagickWand;
 min, max: culong): MagickBooleanType; cdecl; external WandExport;
=======
=======
>>>>>>> origin/fixes_2_2
 const channel: ChannelType; minima, maxima: PCardinal): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageChannelMean(wand: PMagickWand;
 const channel: ChannelType; mean, standard_deviation: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageColormapColor(wand: PMagickWand;
 const index: Cardinal; color: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageExtrema(wand: PMagickWand;
 min, max: Cardinal): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MagickGetImageGreenPrimary(wand: PMagickWand; x, y: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageMatteColor(wand: PMagickWand;
 matte_color: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickGetImagePage(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 width, height: Pculong; x, y: Pclong): MagickBooleanType; cdecl; external WandExport;
function MagickGetImagePixelColor(wand: PMagickWand;
 const x, y: clong; color: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickGetImagePixels(wand: PMagickWand;
 const x, y: clong; const columns, rows: culong;
=======
=======
>>>>>>> origin/fixes_2_2
 width, height: PCardinal; x, y: PInteger): MagickBooleanType; cdecl; external WandExport;
function MagickGetImagePixelColor(wand: PMagickWand;
 const x, y: Integer; color: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickGetImagePixels(wand: PMagickWand;
 const x, y: Integer; const columns, rows: Cardinal;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 const map: PChar; const storage: StorageType; pixels: Pointer): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageRedPrimary(wand: PMagickWand; x, y: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageResolution(wand: PMagickWand; x, y: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickGetImageWhitePoint(wand: PMagickWand; x, y: Pdouble): MagickBooleanType; cdecl; external WandExport;
function MagickHasNextImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickHasPreviousImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickImplodeImage(wand: PMagickWand; const radius: double): MagickBooleanType; cdecl; external WandExport;
function MagickLabelImage(wand: PMagickWand; const _label: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickLevelImage(wand: PMagickWand;
 const black_point, gamma, white_point: double): MagickBooleanType; cdecl; external WandExport;
function MagickLevelImageChannel(wand: PMagickWand;
 const channel: ChannelType; const black_point, gamma, white_point: double
 ): MagickBooleanType; cdecl; external WandExport;
function MagickMagnifyImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickMapImage(wand: PMagickWand;
 const map_wand: PMagickWand; const dither: MagickBooleanType): MagickBooleanType; cdecl; external WandExport;
function MagickMatteFloodfillImage(wand: PMagickWand;
 const opacity: Quantum; const fuzz: double; const bordercolor: PPixelWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const x, y: clong): MagickBooleanType; cdecl; external WandExport;
=======
 const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
>>>>>>> graemeg/fixes_2_2
=======
 const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
>>>>>>> origin/fixes_2_2
function MagickMedianFilterImage(wand: PMagickWand; const radius: double): MagickBooleanType; cdecl; external WandExport;
function MagickMinifyImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickModulateImage(wand: PMagickWand;
 const brightness, saturation, hue: double): MagickBooleanType; cdecl; external WandExport;
function MagickMotionBlurImage(wand: PMagickWand;
 const radius, sigma, angle: double): MagickBooleanType; cdecl; external WandExport;
function MagickNegateImage(wand: PMagickWand; const gray: MagickBooleanType): MagickBooleanType; cdecl; external WandExport;
{
function MagickNegateImageChannel(wand: PMagickWand): MagickBooleanType; external WandExport;
  (MagickWand *,const ChannelType,
    const MagickBooleanType),
  Documentation not found
}
function MagickNewImage(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const columns, rows: culong; const background: PPixelWand): MagickBooleanType; cdecl; external WandExport;
=======
 const columns, rows: Cardinal; const background: PPixelWand): MagickBooleanType; cdecl; external WandExport;
>>>>>>> graemeg/fixes_2_2
=======
 const columns, rows: Cardinal; const background: PPixelWand): MagickBooleanType; cdecl; external WandExport;
>>>>>>> origin/fixes_2_2
function MagickNextImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickNormalizeImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickOilPaintImage(wand: PMagickWand; const radius: double): MagickBooleanType; cdecl; external WandExport;
function MagickPaintOpaqueImage(wand: PMagickWand;
 const target, fill: PPixelWand; const fuzz: double): MagickBooleanType; cdecl; external WandExport;
function MagickPaintTransparentImage(wand: PMagickWand;
 const target: PPixelWand; const opacity: Quantum; const fuzz: double): MagickBooleanType; cdecl; external WandExport;
function MagickPingImage(wand: PMagickWand;
 const filename: PChar): MagickBooleanType; cdecl; external WandExport;
{
function MagickPosterizeImage(wand: PMagickWand;
 const levels: Word; const dither: MagickBooleanType): MagickBooleanType; cdecl; external WandExport;
??
}
function MagickPreviousImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickProfileImage(wand: PMagickWand; const name: PChar;
<<<<<<< HEAD
<<<<<<< HEAD
 const profile: PChar; const length: culong): MagickBooleanType; cdecl; external WandExport;
function MagickQuantizeImage(wand: PMagickWand;
 const number_colors: culong; const colorspace: ColorspaceType;
 const treedepth: culong; const dither, measure_error): MagickBooleanType; cdecl; external WandExport;
function MagickQuantizeImages(wand: PMagickWand;
 const number_colors: culong; const colorspace: ColorspaceType;
 const treedepth: culong; const dither: MagickBooleanType;
=======
=======
>>>>>>> origin/fixes_2_2
 const profile: PChar; const length: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickQuantizeImage(wand: PMagickWand;
 const number_colors: Cardinal; const colorspace: ColorspaceType;
 const treedepth: Cardinal; const dither, measure_error): MagickBooleanType; cdecl; external WandExport;
function MagickQuantizeImages(wand: PMagickWand;
 const number_colors: Cardinal; const colorspace: ColorspaceType;
 const treedepth: Cardinal; const dither: MagickBooleanType;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 const measure_error: MagickBooleanType): MagickBooleanType; cdecl; external WandExport;
function MagickRadialBlurImage(wand: PMagickWand; const angle: double): MagickBooleanType; cdecl; external WandExport;
function MagickRadialBlurImageChannel(wand: PMagickWand;
 const channel: ChannelType; const angle: double): MagickBooleanType; cdecl; external WandExport;
function MagickRaiseImage(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const width, height: culong; const x, y: clong;
=======
 const width, height: Cardinal; const x, y: Integer;
>>>>>>> graemeg/fixes_2_2
=======
 const width, height: Cardinal; const x, y: Integer;
>>>>>>> origin/fixes_2_2
 const raise_: MagickBooleanType): MagickBooleanType; cdecl; external WandExport;
function MagickReadImage(wand: PMagickWand; const filename: PChar
): MagickBooleanType; cdecl; external WandExport;
function MagickReadImageBlob(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const blob: Pointer; const length: clong): MagickBooleanType; cdecl; external WandExport;
=======
 const blob: Pointer; const length: Integer): MagickBooleanType; cdecl; external WandExport;
>>>>>>> graemeg/fixes_2_2
=======
 const blob: Pointer; const length: Integer): MagickBooleanType; cdecl; external WandExport;
>>>>>>> origin/fixes_2_2
//function MagickReadImageFile(wand: PMagickWand; file_: file): MagickBooleanType; external WandExport;

function MagickReduceNoiseImage(wand: PMagickWand; const radius: double): MagickBooleanType; cdecl; external WandExport;
function MagickRemoveImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickResampleImage(wand: PMagickWand;
 const x_resolution, y_resolution: double;
 const filter: FilterTypes; const blur: double): MagickBooleanType; cdecl; external WandExport;
function MagickResizeImage(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const columns, rows: culong; const filter: FilterTypes;
 const blur: double): MagickBooleanType; cdecl; external WandExport;
function MagickRollImage(wand: PMagickWand;
 const x: clong; const y: culong): MagickBooleanType; cdecl; external WandExport;
function MagickRotateImage(wand: PMagickWand; const background: PPixelWand;
 const degrees: double): MagickBooleanType; cdecl; external WandExport;
function MagickSampleImage(wand: PMagickWand; const columns, rows: culong): MagickBooleanType; cdecl; external WandExport;
function MagickScaleImage(wand: PMagickWand; const columns, rows: culong): MagickBooleanType; cdecl; external WandExport;
=======
=======
>>>>>>> origin/fixes_2_2
 const columns, rows: Cardinal; const filter: FilterTypes;
 const blur: double): MagickBooleanType; cdecl; external WandExport;
function MagickRollImage(wand: PMagickWand;
 const x: Integer; const y: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickRotateImage(wand: PMagickWand; const background: PPixelWand;
 const degrees: double): MagickBooleanType; cdecl; external WandExport;
function MagickSampleImage(wand: PMagickWand; const columns, rows: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickScaleImage(wand: PMagickWand; const columns, rows: Cardinal): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MagickSeparateImageChannel(wand: PMagickWand; const channel: ChannelType): MagickBooleanType; cdecl; external WandExport;

// MagickSepiaToneImage(MagickWand *,const double),

function MagickSetImage(wand: PMagickWand; const set_wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageAttribute(wand: PMagickWand; const key, attribute: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageBackgroundColor(wand: PMagickWand; const background: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageBias(wand: PMagickWand; const bias: double): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageBluePrimary(wand: PMagickWand; const x, y: double): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageBorderColor(wand: PMagickWand; const border: PPixelWand): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
<<<<<<< HEAD
function MagickSetImageChannelDepth(wand: PMagickWand; const channel: ChannelType; const depth: culong): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageColormapColor(wand: PMagickWand; const index: culong; const color: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageColorspace(wand: PMagickWand; const colorspace: ColorspaceType): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageCompose(wand: PMagickWand; const compose: CompositeOperator): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageCompression(wand: PMagickWand; const compression: CompressionType): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageCompressionQuality(wand: PMagickWand; const quality: culong): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageDelay(wand: PMagickWand; const delay: culong): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageDepth(wand: PMagickWand; const depth: culong): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageDispose(wand: PMagickWand; const dispose: DisposeType): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageExtent(wand: PMagickWand; const columns, rows: culong): MagickBooleanType; cdecl; external WandExport;
=======
=======
>>>>>>> origin/fixes_2_2
function MagickSetImageChannelDepth(wand: PMagickWand; const channel: ChannelType; const depth: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageColormapColor(wand: PMagickWand; const index: Cardinal; const color: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageColorspace(wand: PMagickWand; const colorspace: ColorspaceType): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageCompose(wand: PMagickWand; const compose: CompositeOperator): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageCompression(wand: PMagickWand; const compression: CompressionType): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageCompressionQuality(wand: PMagickWand; const quality: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageDelay(wand: PMagickWand; const delay: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageDepth(wand: PMagickWand; const depth: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageDispose(wand: PMagickWand; const dispose: DisposeType): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageExtent(wand: PMagickWand; const columns, rows: Cardinal): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MagickSetImageFilename(wand: PMagickWand; const filename: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageFormat(wand: PMagickWand; const format: PChar): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageGamma(wand: PMagickWand; const gamma: Double): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageGreenPrimary(wand: PMagickWand; const x, y: Double): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
<<<<<<< HEAD
function MagickSetImageIndex(wand: PMagickWand; const index: clong): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageInterlaceScheme(wand: PMagickWand; const interlace_scheme: InterlaceType): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageIterations(wand: PMagickWand; const iterations: culong): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageMatteColor(wand: PMagickWand; const matte: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickSetImagePage(wand: PMagickWand; const width, height: culong; const x, y: clong): MagickBooleanType; cdecl; external WandExport;
function MagickSetImagePixels(wand: PMagickWand; const x, y: clong; const columns, rows: culong;
 const map: PChar; const storage: StorageType; const pixels: Pointer): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageProfile(wand: PMagickWand; const name: PChar; const profile: Pointer;
 const length: culong): MagickBooleanType; cdecl; external WandExport;
=======
=======
>>>>>>> origin/fixes_2_2
function MagickSetImageIndex(wand: PMagickWand; const index: Integer): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageInterlaceScheme(wand: PMagickWand; const interlace_scheme: InterlaceType): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageIterations(wand: PMagickWand; const iterations: Cardinal): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageMatteColor(wand: PMagickWand; const matte: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickSetImagePage(wand: PMagickWand; const width, height: Cardinal; const x, y: Integer): MagickBooleanType; cdecl; external WandExport;
function MagickSetImagePixels(wand: PMagickWand; const x, y: Integer; const columns, rows: Cardinal;
 const map: PChar; const storage: StorageType; const pixels: Pointer): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageProfile(wand: PMagickWand; const name: PChar; const profile: Pointer;
 const length: Cardinal): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function MagickSetImageRedPrimary(wand: PMagickWand; const x, y: Double): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageRenderingIntent(wand: PMagickWand; const rendering_intent: RenderingIntent
 ): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageResolution(wand: PMagickWand; const x_resolution, y_resolution: double
 ): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
<<<<<<< HEAD
function MagickSetImageScene(wand: PMagickWand; const scene: culong): MagickBooleanType; cdecl; external WandExport;
=======
function MagickSetImageScene(wand: PMagickWand; const scene: Cardinal): MagickBooleanType; cdecl; external WandExport;
>>>>>>> graemeg/fixes_2_2
=======
function MagickSetImageScene(wand: PMagickWand; const scene: Cardinal): MagickBooleanType; cdecl; external WandExport;
>>>>>>> origin/fixes_2_2
function MagickSetImageType(wand: PMagickWand; const image_type: ImageType
 ): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageUnits(wand: PMagickWand; const units: ResolutionType
 ): MagickBooleanType; cdecl; external WandExport;
//function MagickSetImageVirtualPixelMethod(wand: PMagickWand;
// const method: VirtualPixelMethod): MagickBooleanType; cdecl; external WandExport;
function MagickSetImageWhitePoint(wand: PMagickWand; const x, y: double
 ): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
<<<<<<< HEAD
function MagickShadowImage(wand: PMagickWand; const radius, sigma: double; const x, y: clong
=======
function MagickShadowImage(wand: PMagickWand; const radius, sigma: double; const x, y: Integer
>>>>>>> graemeg/fixes_2_2
=======
function MagickShadowImage(wand: PMagickWand; const radius, sigma: double; const x, y: Integer
>>>>>>> origin/fixes_2_2
 ): MagickBooleanType; cdecl; external WandExport;
function MagickSharpenImage(wand: PMagickWand; const radius, sigma: double
 ): MagickBooleanType; cdecl; external WandExport;
function MagickSharpenImageChannel(wand: PMagickWand; const channel: ChannelType;
 const radius, sigma: double): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
<<<<<<< HEAD
function MagickShaveImage(wand: PMagickWand; const columns, rows: culong
=======
function MagickShaveImage(wand: PMagickWand; const columns, rows: Cardinal
>>>>>>> graemeg/fixes_2_2
=======
function MagickShaveImage(wand: PMagickWand; const columns, rows: Cardinal
>>>>>>> origin/fixes_2_2
 ): MagickBooleanType; cdecl; external WandExport;
function MagickShearImage(wand: PMagickWand; const background: PPixelWand;
 const x_shear, y_shear: double): MagickBooleanType; cdecl; external WandExport;
function MagickSigmoidalContrastImage(wand: PMagickWand;
 const sharpen: MagickBooleanType; const alpha, beta: double
 ): MagickBooleanType; cdecl; external WandExport;
function MagickSigmoidalContrastImageChannel(wand: PMagickWand;
 const channel: ChannelType; const sharpen: MagickBooleanType; const alpha, beta: double
 ): MagickBooleanType; cdecl; external WandExport;
function MagickSolarizeImage(wand: PMagickWand; const threshold: double
 ): MagickBooleanType; cdecl; external WandExport;
<<<<<<< HEAD
<<<<<<< HEAD
function MagickSpliceImage(wand: PMagickWand; const width, height: culong; const x, y: clong
=======
function MagickSpliceImage(wand: PMagickWand; const width, height: Cardinal; const x, y: Integer
>>>>>>> graemeg/fixes_2_2
=======
function MagickSpliceImage(wand: PMagickWand; const width, height: Cardinal; const x, y: Integer
>>>>>>> origin/fixes_2_2
 ): MagickBooleanType; cdecl; external WandExport;
function MagickSpreadImage(wand: PMagickWand; const radius: double): MagickBooleanType; cdecl; external WandExport;
function MagickStripImage(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickSwirlImage(wand: PMagickWand; const degrees: double): MagickBooleanType; cdecl; external WandExport;
function MagickTintImage(wand: PMagickWand; const tint: PPixelWand; const opacity: PPixelWand
 ): MagickBooleanType; cdecl; external WandExport;
function MagickThresholdImage(wand: PMagickWand; const threshold: double): MagickBooleanType; cdecl; external WandExport;
function MagickThresholdImageChannel(wand: PMagickWand; const channel: ChannelType;
 const threshold: double): MagickBooleanType; cdecl; external WandExport;
function MagickTrimImage(wand: PMagickWand; const fuzz: double): MagickBooleanType; cdecl; external WandExport;
function MagickUnsharpMaskImage(wand: PMagickWand;
 const radius, sigma, amount, threshold: double): MagickBooleanType; cdecl; external WandExport;
function MagickUnsharpMaskImageChannel(wand: PMagickWand;
 const channel: ChannelType; const radius, sigma, amount, threshold: double
 ): MagickBooleanType; cdecl; external WandExport;
function MagickWaveImage(wand: PMagickWand; const amplitude, wave_length: double
 ): MagickBooleanType; cdecl; external WandExport;
function MagickWhiteThresholdImage(wand: PMagickWand; const threshold: PPixelWand): MagickBooleanType; cdecl; external WandExport;
function MagickWriteImage(wand: PMagickWand;
 const filename: PChar): MagickBooleanType; cdecl; external WandExport;
//  MagickWriteImageFile(MagickWand *,FILE *),
function MagickWriteImages(wand: PMagickWand;
 const filename: PChar; const adjoin: MagickBooleanType
 ): MagickBooleanType; cdecl; external WandExport;
//  MagickWriteImagesFile(MagickWand *,FILE *);


function MagickSetImageProgressMonitor(wand: PMagickWand;
 const progress_monitor: MagickProgressMonitor; client_data: Pointer): MagickProgressMonitor; cdecl; external WandExport;

function MagickGetImageSize(wand: PMagickWand): MagickSizeType; cdecl; external WandExport;

function MagickAppendImages(wand: PMagickWand; const stack: MagickBooleanType): PMagickWand; cdecl; external WandExport;
function MagickAverageImages(wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function MagickCoalesceImages(wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function MagickCombineImages(wand: PMagickWand; para2: ChannelType): PMagickWand; cdecl; external WandExport;
function MagickCompareImageChannels(wand: PMagickWand;
 const reference: PMagickWand; const channel: ChannelType; const metric: MetricType; distortion: PDouble
 ): PMagickWand; cdecl; external WandExport;
function MagickCompareImages(wand: PMagickWand;
 const reference: PMagickWand; const metric: MetricType; distortion: PDouble
 ): PMagickWand; cdecl; external WandExport;
function MagickDeconstructImages(wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function MagickFlattenImages(wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function MagickFxImage(wand: PMagickWand; const expression: PChar): PMagickWand; cdecl; external WandExport;
function MagickFxImageChannel(wand: PMagickWand; const channel: ChannelType; const expression: PChar
 ): PMagickWand; cdecl; external WandExport;
function MagickGetImage(wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function MagickGetImageRegion(wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const width, height: culong; const x, y: clong
 ): PMagickWand; cdecl; external WandExport;
function MagickMorphImages(wand: PMagickWand; const number_frames: culong): PMagickWand; cdecl; external WandExport;
=======
 const width, height: Cardinal; const x, y: Integer
 ): PMagickWand; cdecl; external WandExport;
function MagickMorphImages(wand: PMagickWand; const number_frames: Cardinal): PMagickWand; cdecl; external WandExport;
>>>>>>> graemeg/fixes_2_2
=======
 const width, height: Cardinal; const x, y: Integer
 ): PMagickWand; cdecl; external WandExport;
function MagickMorphImages(wand: PMagickWand; const number_frames: Cardinal): PMagickWand; cdecl; external WandExport;
>>>>>>> origin/fixes_2_2
function MagickMosaicImages(wand: PMagickWand): PMagickWand; cdecl; external WandExport;
//function MagickMontageImage(wand: PMagickWand;
// const drawing_wand: DrawingWand; const tile_geometry: PChar;
// const thumbnail_geometry: PChar; const mode: MontageMode; const frame: PChar
// ): PMagickWand; cdecl; external WandExport;
function MagickPreviewImages(wand: PMagickWand; const preview: PreviewType): PMagickWand; cdecl; external WandExport;
function MagickSteganoImage(wand: PMagickWand; const watermark_wand: PMagickWand;
<<<<<<< HEAD
<<<<<<< HEAD
 const offset: clong): PMagickWand; cdecl; external WandExport;
=======
 const offset: Integer): PMagickWand; cdecl; external WandExport;
>>>>>>> graemeg/fixes_2_2
=======
 const offset: Integer): PMagickWand; cdecl; external WandExport;
>>>>>>> origin/fixes_2_2
function MagickStereoImage(wand: PMagickWand; const offset_wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function MagickTextureImage(wand: PMagickWand; const texture_wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function MagickTransformImage(wand: PMagickWand; const crop, geometry: PChar): PMagickWand; cdecl; external WandExport;
function NewMagickWandFromImage(para1: PImage): PMagickWand; cdecl; external WandExport;

<<<<<<< HEAD
<<<<<<< HEAD
function MagickGetImageHistogram(wand: PMagickWand; number_colors: Pculong): PPPixelWand; cdecl; external  WandExport;
=======
function MagickGetImageHistogram(wand: PMagickWand; number_colors: PCardinal): PPPixelWand; cdecl; external  WandExport;
>>>>>>> graemeg/fixes_2_2
=======
function MagickGetImageHistogram(wand: PMagickWand; number_colors: PCardinal): PPPixelWand; cdecl; external  WandExport;
>>>>>>> origin/fixes_2_2

function MagickGetImageRenderingIntent(wand: PMagickWand): RenderingIntent; cdecl; external WandExport;

function MagickGetImageUnits(wand: PMagickWand): ResolutionType; cdecl; external WandExport;

<<<<<<< HEAD
<<<<<<< HEAD
function MagickGetImageBlob(wand: PMagickWand; length: Pcsize_t): PByte; cdecl; external WandExport;
function MagickGetImagesBlob(wand: PMagickWand; length: Pcsize_t): PByte; cdecl; external WandExport;
function MagickGetImageProfile(wand: PMagickWand; name: PChar; length: Pcsize_t): PByte; cdecl; external WandExport;
function MagickRemoveImageProfile(wand: PMagickWand; name: PChar; length: Pcsize_t): PByte; cdecl; external WandExport;

function MagickGetImageColors(wand: PMagickWand): culong; cdecl; external WandExport;
function MagickGetImageCompressionQuality(wand: PMagickWand): culong; cdecl; external WandExport;
function MagickGetImageDelay(wand: PMagickWand): culong; cdecl; external WandExport;
function MagickGetImageChannelDepth(wand: PMagickWand; const channel: ChannelType): culong; cdecl; external WandExport;
function MagickGetImageDepth(wand: PMagickWand): culong; cdecl; external WandExport;
function MagickGetImageHeight(wand: PMagickWand): culong; cdecl; external WandExport;
function MagickGetImageIterations(wand: PMagickWand): culong; cdecl; external WandExport;
function MagickGetImageScene(wand: PMagickWand): culong; cdecl; external WandExport;
function MagickGetImageWidth(wand: PMagickWand): culong; cdecl; external WandExport;
function MagickGetNumberImages(wand: PMagickWand): culong; cdecl; external WandExport;
=======
=======
>>>>>>> origin/fixes_2_2
function MagickGetImageBlob(wand: PMagickWand; length: Psize_t): PByte; cdecl; external WandExport;
function MagickGetImagesBlob(wand: PMagickWand; length: Psize_t): PByte; cdecl; external WandExport;
function MagickGetImageProfile(wand: PMagickWand; name: PChar; length: Psize_t): PByte; cdecl; external WandExport;
function MagickRemoveImageProfile(wand: PMagickWand; name: PChar; length: Psize_t): PByte; cdecl; external WandExport;

function MagickGetImageColors(wand: PMagickWand): Cardinal; cdecl; external WandExport;
function MagickGetImageCompressionQuality(wand: PMagickWand): Cardinal; cdecl; external WandExport;
function MagickGetImageDelay(wand: PMagickWand): Cardinal; cdecl; external WandExport;
function MagickGetImageChannelDepth(wand: PMagickWand; const channel: ChannelType): Cardinal; cdecl; external WandExport;
function MagickGetImageDepth(wand: PMagickWand): Cardinal; cdecl; external WandExport;
function MagickGetImageHeight(wand: PMagickWand): Cardinal; cdecl; external WandExport;
function MagickGetImageIterations(wand: PMagickWand): Cardinal; cdecl; external WandExport;
function MagickGetImageScene(wand: PMagickWand): Cardinal; cdecl; external WandExport;
function MagickGetImageWidth(wand: PMagickWand): Cardinal; cdecl; external WandExport;
function MagickGetNumberImages(wand: PMagickWand): Cardinal; cdecl; external WandExport;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

//function MagickGetImageVirtualPixelMethod(wand: PMagickWand): VirtualPixelMethod; cdecl; external WandExport;

{
  Deprecated methods.
}
{extern WandExport char
  *MagickDescribeImage(MagickWand *);

extern WandExport MagickBooleanType
  MagickOpaqueImage(MagickWand *,const PixelWand *,const PixelWand *,
    const double),
  MagickSetImageOption(MagickWand *,const char *,const char *,const char *),
  MagickTransparentImage(MagickWand *,const PixelWand *,const Quantum,
    const double);

extern WandExport MagickWand
  *MagickRegionOfInterestImage(MagickWand *,const unsigned long,
	  const unsigned long,const long,const long);

extern WandExport unsigned char
  *MagickWriteImageBlob(MagickWand *,size_t *);
}
