{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by Marco van de Voort
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

    Note these functions were in the windows unit in older versions

 **********************************************************************}
unit commdlg;

interface
{$mode delphi} // interface
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef Win64}
  {$packrecords 1}
{$endif}

{$ifdef FPC_OS_UNICODE}
  {$define UNICODE}
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
uses windows;

Type
     LPOFNHOOKPROC=function (h:HWND; u:UINT; w:WPARAM; l:LPARAM):  UINT_PTR; stdcall;
     OPENFILENAMEA_NT4 = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          lpstrFilter : LPCTSTR;
          lpstrCustomFilter : LPTSTR;
          nMaxCustFilter : DWORD;
          nFilterIndex : DWORD;
          lpstrFile : LPTSTR;
          nMaxFile : DWORD;
          lpstrFileTitle : LPTSTR;
          nMaxFileTitle : DWORD;
          lpstrInitialDir : LPCTSTR;
          lpstrTitle : LPCTSTR;
          Flags : DWORD;
          nFileOffset : WORD;
          nFileExtension : WORD;
          lpstrDefExt : LPCTSTR;
          lCustData : LPARAM;
          lpfnHook : LPOFNHOOKPROC;
          lpTemplateName : LPCTSTR;
       end;
     LPOPENFILENAMEA_NT4 = ^OPENFILENAMEA_NT4;
     TOPENFILENAMEA_NT4 = OPENFILENAMEA_NT4;
     POPENFILENAMEA_NT4 = ^OPENFILENAMEA_NT4;
	 
     OPENFILENAMEW_NT4 = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          lpstrFilter : LPWSTR;
          lpstrCustomFilter : LPWSTR;
          nMaxCustFilter : DWORD;
          nFilterIndex : DWORD;
          lpstrFile : LPWSTR;
          nMaxFile : DWORD;
          lpstrFileTitle : LPWSTR;
          nMaxFileTitle : DWORD;
          lpstrInitialDir : LPWSTR;
          lpstrTitle : LPWSTR;
          Flags : DWORD;
          nFileOffset : WORD;
          nFileExtension : WORD;
          lpstrDefExt : LPWSTR;
          lCustData : LPARAM;
          lpfnHook : LPOFNHOOKPROC;
          lpTemplateName : LPWSTR;
       end;
     LPOPENFILENAMEW_NT4 = ^OPENFILENAMEW_NT4;
     TOPENFILENAMEW_NT4 = OPENFILENAMEW_NT4;
     POPENFILENAMEW_NT4 = ^OPENFILENAMEW_NT4;
	 {$IFDEF UNICODE}  
	  LPOPENFILENAME_NT4 = ^OPENFILENAMEW_NT4;
      TOPENFILENAME_NT4 = OPENFILENAMEW_NT4;
      POPENFILENAME_NT4 = ^OPENFILENAMEW_NT4;	  
      OPENFILENAME_NT4 = OPENFILENAMEW_NT4;	  
	 {$else}
	  LPOPENFILENAME_NT4 = ^OPENFILENAMEA_NT4;
      TOPENFILENAME_NT4 = OPENFILENAMEA_NT4;
      POPENFILENAME_NT4 = ^OPENFILENAMEA_NT4;
      OPENFILENAME_NT4 = OPENFILENAMEA_NT4;	  
	 {$endif}
     
	 OPENFILENAMEA = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          lpstrFilter : LPCTSTR;
          lpstrCustomFilter : LPTSTR;
          nMaxCustFilter : DWORD;
          nFilterIndex : DWORD;
          lpstrFile : LPTSTR;
          nMaxFile : DWORD;
          lpstrFileTitle : LPTSTR;
          nMaxFileTitle : DWORD;
          lpstrInitialDir : LPCTSTR;
          lpstrTitle : LPCTSTR;
          Flags : DWORD;
          nFileOffset : WORD;
          nFileExtension : WORD;
          lpstrDefExt : LPCTSTR;
          lCustData : LPARAM;
          lpfnHook : LPOFNHOOKPROC;
          lpTemplateName : LPCTSTR;
          pvReserved : pointer;
          dwreserved : dword;
          FlagsEx    : dword; 
       end;
     LPOPENFILENAMEA = ^OPENFILENAMEA;
     TOPENFILENAMEA = OPENFILENAMEA;
     POPENFILENAMEA = ^OPENFILENAMEA;

	 OPENFILENAMEW = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          lpstrFilter : LPWSTR;
          lpstrCustomFilter : LPWSTR;
          nMaxCustFilter : DWORD;
          nFilterIndex : DWORD;
          lpstrFile : LPWSTR;
          nMaxFile : DWORD;
          lpstrFileTitle : LPWSTR;
          nMaxFileTitle : DWORD;
          lpstrInitialDir : LPWSTR;
          lpstrTitle : LPWSTR;
          Flags : DWORD;
          nFileOffset : WORD;
          nFileExtension : WORD;
          lpstrDefExt : LPWSTR;
          lCustData : LPARAM;
          lpfnHook : LPOFNHOOKPROC;
          lpTemplateName : LPWSTR;
          pvReserved : pointer;
          dwreserved : dword;
          FlagsEx    : dword; 
       end;
     LPOPENFILENAMEW = ^OPENFILENAMEW;
     TOPENFILENAMEW = OPENFILENAMEW;
     POPENFILENAMEW = ^OPENFILENAMEW;	 
	 
     OFNOTIFYA = record
          hdr : NMHDR;
          lpOFN : LPOPENFILENAME;
          pszFile : LPTSTR;
       end;
     LPOFNOTIFYA = ^OFNOTIFYA;
     _OFNOTIFYA = OFNOTIFYA;
     TOFNOTIFYA = OFNOTIFYA;
     POFNOTIFYA = ^OFNOTIFYA;

     OFNOTIFYW = record
          hdr : NMHDR;
          lpOFN : LPOPENFILENAMEW;
          pszFile : LPWSTR;
       end;
     LPOFNOTIFYW = ^OFNOTIFYW;
     _OFNOTIFYW = OFNOTIFYW;
     TOFNOTIFYW = OFNOTIFYW;
     POFNOTIFYW = ^OFNOTIFYW;
     {$ifdef unicode}
       OFNOTIFY = OFNOTIFYW;
       _OFNOTIFY = OFNOTIFYW;
       TOFNOTIFY = OFNOTIFYW;
       POFNOTIFY = ^OFNOTIFYW;
      LPOFNOTIFY = POFNOTIFYW;	   
     {$else}
       OFNOTIFY = OFNOTIFYA;
       _OFNOTIFY = OFNOTIFYA;
       TOFNOTIFY = OFNOTIFYA;
       POFNOTIFY = ^OFNOTIFYA;
	   LPOFNOTIFY = POFNOTIFYA;	   
     {$endif}

     OFNOTIFYEXA = record
          hdr : NMHDR;
          lpOFN : LPOPENFILENAMEA;
          pszFile : pointer;
		  pidl	  : pointer;
       end;
     LPOFNOTIFYEXA = ^OFNOTIFYEXA;
     _OFNOTIFYEXA = OFNOTIFYEXA;
     TOFNOTIFYEXA = OFNOTIFYEXA;
     POFNOTIFYEXA = ^OFNOTIFYEXA;

     OFNOTIFYEXW = record
          hdr : NMHDR;
          lpOFN : LPOPENFILENAMEW;
          pszFile : pointer;
		  pidl	  : pointer;
       end;
     LPOFNOTIFYEXW = ^OFNOTIFYEXW;
     _OFNOTIFYEXW = OFNOTIFYEXW;
     TOFNOTIFYEXW = OFNOTIFYEXW;
     POFNOTIFYEXW = ^OFNOTIFYEXW;
     {$ifdef unicode}
       OFNOTIFYEX = OFNOTIFYEXW;
       _OFNOTIFYEX = OFNOTIFYEXW;
       TOFNOTIFYEX = OFNOTIFYEXW;
       POFNOTIFYEX = ^OFNOTIFYEXW;
     {$else}
       OFNOTIFYEX = OFNOTIFYEXA;
       _OFNOTIFYEX = OFNOTIFYEXA;
       TOFNOTIFYEX = OFNOTIFYEXA;
       POFNOTIFYEX = ^OFNOTIFYEXA;
     {$endif}

   {CHOOSECOLOR = record confilcts with function ChooseColor }
    TCHOOSECOLORA = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HWND;
          rgbResult : COLORREF;
          lpCustColors : ^COLORREF;
          Flags : DWORD;
          lCustData : LPARAM;
          lpfnHook : LPCCHOOKPROC;
          lpTemplateName : LPCTSTR;		  
       end;
   LPCHOOSECOLORA = ^TCHOOSECOLORA;
   PCHOOSECOLORA  = ^TCHOOSECOLORA;
   _CHOOSECOLORA   = TCHOOSECOLORA;
   
   TCHOOSECOLORW  = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HWND;
          rgbResult : COLORREF;
          lpCustColors : ^COLORREF;
          Flags : DWORD;
          lCustData : LPARAM;
          lpfnHook : LPCCHOOKPROC;
          lpTemplateName : LPCWSTR;
       end;
     LPCHOOSECOLORW = ^TCHOOSECOLORW;
     PCHOOSECOLORW  = ^TCHOOSECOLORW;
	 _CHOOSECOLORW   = TCHOOSECOLORW;
    {$ifdef unicode}       
       _CHOOSECOLOR = TCHOOSECOLORW;
       TCHOOSECOLOR = TCHOOSECOLORW;
       PCHOOSECOLOR = ^TCHOOSECOLORW;
       LPCHOOSECOLOR = PCHOOSECOLORW;	   
     {$else}       
       _CHOOSECOLOR = TCHOOSECOLORA;
       TCHOOSECOLOR = TCHOOSECOLORA;
       PCHOOSECOLOR = ^TCHOOSECOLORA;
       LPCHOOSECOLOR = PCHOOSECOLORA;	   	   
     {$endif}
  
    FINDREPLACEA = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          Flags : DWORD;
          lpstrFindWhat : LPTSTR;
          lpstrReplaceWith : LPTSTR;
          wFindWhatLen : WORD;
          wReplaceWithLen : WORD;
          lCustData : LPARAM;
          lpfnHook : LPFRHOOKPROC;
          lpTemplateName : LPCTSTR;
       end;
     LPFINDREPLACEA = ^FINDREPLACEA;
     TFINDREPLACEA = FINDREPLACEA;
     PFINDREPLACEA = ^FINDREPLACEA;
     FINDREPLACEW = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hInstance : HINST;
          Flags : DWORD;
          lpstrFindWhat : LPWSTR;
          lpstrReplaceWith : LPWSTR;
          wFindWhatLen : WORD;
          wReplaceWithLen : WORD;
          lCustData : LPARAM;
          lpfnHook : LPFRHOOKPROC;
          lpTemplateName : LPWSTR;
       end;
     LPFINDREPLACEW = ^FINDREPLACEW;
     TFINDREPLACEW = FINDREPLACEW;
     PFINDREPLACEW = ^FINDREPLACEW;
     {$ifdef unicode}       
       _FINDREPLACE = FINDREPLACEW;
       TFINDREPLACE = FINDREPLACEW;
       PFINDREPLACE = ^FINDREPLACEW;
       LPFINDREPLACE = PFINDREPLACEW;	   
     {$else}       
       _FINDREPLACE = FINDREPLACEA;
       TFINDREPLACE = FINDREPLACEA;
       PFINDREPLACE = ^FINDREPLACEA;
       LPFINDREPLACE = PFINDREPLACEA;	   	   
     {$endif}
	
     LOGFONTA = record
          lfHeight : LONG;
          lfWidth : LONG;
          lfEscapement : LONG;
          lfOrientation : LONG;
          lfWeight : LONG;
          lfItalic : BYTE;
          lfUnderline : BYTE;
          lfStrikeOut : BYTE;
          lfCharSet : BYTE;
          lfOutPrecision : BYTE;
          lfClipPrecision : BYTE;
          lfQuality : BYTE;
          lfPitchAndFamily : BYTE;
          lfFaceName : array[0..(LF_FACESIZE)-1] of TCHAR;
       end;
     
     TLOGFONTA = LOGFONTA;
     PLOGFONTA = ^LOGFONTA;
     LPLOGFONTA = PLOGFONTA;

     LOGFONTW = record
       lfHeight: LONG;
       lfWidth: LONG;
       lfEscapement: LONG;
       lfOrientation: LONG;
       lfWeight: LONG;
       lfItalic: BYTE;
       lfUnderline: BYTE;
       lfStrikeOut: BYTE;
       lfCharSet: BYTE;
       lfOutPrecision: BYTE;
       lfClipPrecision: BYTE;
       lfQuality: BYTE;
       lfPitchAndFamily: BYTE;
       lfFaceName: array [0..LF_FACESIZE - 1] of WCHAR;
     end;
     LPLOGFONTW = ^LOGFONTW;
     NPLOGFONTW = ^LOGFONTW;
     TLogFontW = LOGFONTW;
     PLogFontW = ^TLogFontW;
     {$ifdef unicode}
       LOGFONT = LOGFONTW;
       _LOGFONT = LOGFONTW;
       TLOGFONT = LOGFONTW;
       PLOGFONT = ^LOGFONTW;
       LPLOGFONT = PLOGFONTW;	   
     {$else}
       LOGFONT = LOGFONTA;
       _LOGFONT = LOGFONTA;
       TLOGFONT = LOGFONTA;
       PLOGFONT = ^LOGFONTA;
       LPLOGFONT = PLOGFONTA;	   	   
     {$endif}
 
	 _CHOOSEFONTA = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDC : HDC;
          lpLogFont : LPLOGFONTA;
          iPointSize : WINT;
          Flags : DWORD;
          rgbColors : DWORD;
          lCustData : LPARAM;
          lpfnHook : LPCFHOOKPROC;
          lpTemplateName : LPCTSTR;
          hInstance : HINST;
          lpszStyle : LPTSTR;
          nFontType : WORD;
          ___MISSING_ALIGNMENT__ : WORD;
          nSizeMin : WINT;
          nSizeMax : WINT;
       end;
     LPCHOOSEFONTA = ^TCHOOSEFONTA;
     PCHOOSEFONTA = ^TCHOOSEFONTA;
     TCHOOSEFONTA = _CHOOSEFONTA;	 
	 _CHOOSEFONTW = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDC : HDC;
          lpLogFont : LPLOGFONTW;
          iPointSize : WINT;
          Flags : DWORD;
          rgbColors : DWORD;
          lCustData : LPARAM;
          lpfnHook : LPCFHOOKPROC;
          lpTemplateName : LPWSTR;
          hInstance : HINST;
          lpszStyle : LPWSTR;
          nFontType : WORD;
          ___MISSING_ALIGNMENT__ : WORD;
          nSizeMin : WINT;
          nSizeMax : WINT;
       end;
     LPCHOOSEFONTW = ^TCHOOSEFONTW;
     PCHOOSEFONTW = ^TCHOOSEFONTW;
	 TCHOOSEFONTW = _CHOOSEFONTW;	 
    {$ifdef unicode}     
       _choosefont = TchoosefontW;
       Tchoosefont = TchoosefontW;
       Pchoosefont = ^_choosefontW;
       LPchoosefont = PchoosefontW;	   
     {$else}
     
       _choosefont = TchoosefontA;
       Tchoosefont = TchoosefontA;
       Pchoosefont = ^_choosefontA;
       LPchoosefont = PchoosefontA;	   	   
     {$endif}
	 DEVNAMES = record
          wDriverOffset : WORD;
          wDeviceOffset : WORD;
          wOutputOffset : WORD;
          wDefault : WORD;
       end;
     LPDEVNAMES = ^DEVNAMES;
     tagDEVNAMES = DEVNAMES;
     TDEVNAMES = DEVNAMES;
     PDEVNAMES = ^DEVNAMES;
    
<<<<<<< HEAD
<<<<<<< HEAD
	TPRINTDLGA = record
=======
	TPRINTDLGA = packed record
>>>>>>> graemeg/fixes_2_2
=======
	TPRINTDLGA = packed record
>>>>>>> origin/fixes_2_2
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDevMode : HANDLE;
          hDevNames : HANDLE;
          hDC : HDC;
          Flags : DWORD;
          nFromPage : WORD;
          nToPage : WORD;
          nMinPage : WORD;
          nMaxPage : WORD;
          nCopies : WORD;
          hInstance : HINST;
<<<<<<< HEAD
<<<<<<< HEAD
          lCustData : LPARAM;
=======
          lCustData : DWORD;
>>>>>>> graemeg/fixes_2_2
=======
          lCustData : DWORD;
>>>>>>> origin/fixes_2_2
          lpfnPrintHook : LPPRINTHOOKPROC;
          lpfnSetupHook : LPSETUPHOOKPROC;
          lpPrintTemplateName : LPCTSTR;
          lpSetupTemplateName : LPCTSTR;
          hPrintTemplate : HANDLE;
          hSetupTemplate : HANDLE;
       end;
     LPPRINTDLGA = ^TPRINTDLGA;
     PPRINTDLGA = ^TPRINTDLGA;
     _PRINTDLGA = TPRINTDLGA; 
     tagPDA = TPRINTDLGA;
     TPDA = TPRINTDLGA;
     PPDA = ^TPRINTDLGA;
	 	 
<<<<<<< HEAD
<<<<<<< HEAD
     TPRINTDLGW = record
=======
     TPRINTDLGW = packed record
>>>>>>> graemeg/fixes_2_2
=======
     TPRINTDLGW = packed record
>>>>>>> origin/fixes_2_2
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDevMode : HANDLE;
          hDevNames : HANDLE;
          hDC : HDC;
          Flags : DWORD;
          nFromPage : WORD;
          nToPage : WORD;
          nMinPage : WORD;
          nMaxPage : WORD;
          nCopies : WORD;
          hInstance : HINST;
<<<<<<< HEAD
<<<<<<< HEAD
          lCustData : LPARAM;
=======
          lCustData : DWORD;
>>>>>>> graemeg/fixes_2_2
=======
          lCustData : DWORD;
>>>>>>> origin/fixes_2_2
          lpfnPrintHook : LPPRINTHOOKPROC;
          lpfnSetupHook : LPSETUPHOOKPROC;
          lpPrintTemplateName : LPWSTR;
          lpSetupTemplateName : LPWSTR;
          hPrintTemplate : HANDLE;
          hSetupTemplate : HANDLE;
       end;
     LPPRINTDLGW = ^TPRINTDLGW;
     PPRINTDLGW = ^TPRINTDLGW;
     _PRINTDLGW = TPRINTDLGW; 
     tagPDW = TPRINTDLGW;
     TPDW = TPRINTDLGW;
     PPDW = ^TPRINTDLGW;
     {$ifndef unicode}
	  LPPRINTDLG = ^TPRINTDLGA;
      PPRINTDLG = ^TPRINTDLGA;
      _PRINTDLG = TPRINTDLGA; 
      tagPD = TPRINTDLGA;
      TPD = TPRINTDLGA;
      PPD = ^TPRINTDLGA;
	 {$else}
	  LPPRINTDLG = ^TPRINTDLGW;
      PPRINTDLG = ^TPRINTDLGW;
      _PRINTDLG = TPRINTDLGW; 
      tagPD = TPRINTDLGW;
      TPD = TPRINTDLGW;
      PPD = ^TPRINTDLGW;
	 {$endif}
	 
	TPAGESETUPDLGA = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDevMode : HGLOBAL;
          hDevNames : HGLOBAL;
          Flags : DWORD;
          ptPaperSize : POINT;
          rtMinMargin : RECT;
          rtMargin : RECT;
          hInstance : HINST;
          lCustData : LPARAM;
          lpfnPageSetupHook : LPPAGESETUPHOOK;
          lpfnPagePaintHook : LPPAGEPAINTHOOK;
          lpPageSetupTemplateName : LPCTSTR;
          hPageSetupTemplate : HGLOBAL;
       end;
     LPPAGESETUPDLGA = ^TPAGESETUPDLGA;
     PPAGESETUPDLGA = ^TPAGESETUPDLGA;
     _PAGESETUPDLGA = TPAGESETUPDLGA;
     tagPSDA = TPAGESETUPDLGA;
     TPSDA = TPAGESETUPDLGA;
     PPSDA = ^TPAGESETUPDLGA;
	 
	 TPAGESETUPDLGW = record
          lStructSize : DWORD;
          hwndOwner : HWND;
          hDevMode : HGLOBAL;
          hDevNames : HGLOBAL;
          Flags : DWORD;
          ptPaperSize : POINT;
          rtMinMargin : RECT;
          rtMargin : RECT;
          hInstance : HINST;
          lCustData : LPARAM;
          lpfnPageSetupHook : LPPAGESETUPHOOK;
          lpfnPagePaintHook : LPPAGEPAINTHOOK;
          lpPageSetupTemplateName : LPWSTR;
          hPageSetupTemplate : HGLOBAL;
       end;
     LPPAGESETUPDLGW = ^TPAGESETUPDLGW;
     PPAGESETUPDLGW = ^TPAGESETUPDLGW;
     _PAGESETUPDLGW = TPAGESETUPDLGW;
     tagPSDW = TPAGESETUPDLGW;
     TPSDW = TPAGESETUPDLGW;
     PPSDW = ^TPAGESETUPDLGW;

    {$ifdef unicode}
       
       _PAGESETUPDLG = TPAGESETUPDLGW;
       TPAGESETUPDLG = TPAGESETUPDLGW;
       PPAGESETUPDLG = ^TPAGESETUPDLGW;
       LPPAGESETUPDLG = PPAGESETUPDLGW;	   
     {$else}
       
       _PAGESETUPDLG = TPAGESETUPDLGA;
       TPAGESETUPDLG = TPAGESETUPDLGA;
       PPAGESETUPDLG = ^TPAGESETUPDLGA;
       LPPAGESETUPDLG = PPAGESETUPDLGA;	   	   
     {$endif}
	 	 
// todo  tagpdexa, psdax 	
	
    IPrintDialogCallback= interface(IUnknown)
	    ['{5852A2C3-6530-11D1-B6A3-0000F8757BF9}']
         function InitDone:HResult;Stdcall;
         function SelectionDone:HResult;Stdcall;		 
         function handleMessage(hdlg:HWND;uMsg:UINT;wparm:WPARAM;Lparm:LPARAM;pres :PLRESULT):HRESULT;StdCall;
        end;
	IPrintDialogServices= interface (IUnknown) 
	    ['{509AAEDA-5639-11D1-B6A1-0000F8757BF9}']
		 function GetCurrentDevMode(pDevMode: LPDEVMODE;pcbSize:PUINT):HResult;Stdcall;
         function GetCurrentPrinterName(pPrinterName:LPWSTR;pcchSize:PUINT):HREsult;Stdcall;
         function GetCurrentPortName(PortName:LPWSTR;pcchSize:PUINT):HREsult;Stdcall;		 
		end;
Const
  comctl32 = 'comctl32.dll';

{$ifndef unicode}
function GetOpenFileName(_para1:LPOPENFILENAME):WINBOOL; stdcall; external 'comdlg32' name 'GetOpenFileNameA';
function GetSaveFileName(_para1:LPOPENFILENAME):WINBOOL; stdcall; external 'comdlg32' name 'GetSaveFileNameA';
function GetFileTitle(_para1:LPCSTR; _para2:LPSTR; _para3:WORD):integer; stdcall; external 'comdlg32' name 'GetFileTitleA';
function ChooseColor(_para1:LPCHOOSECOLOR):WINBOOL; stdcall; external 'comdlg32' name 'ChooseColorA';
function FindText(_para1:LPFINDREPLACE):HWND; stdcall; external 'comdlg32' name 'FindTextA';
function ReplaceText(_para1:LPFINDREPLACE):HWND; stdcall; external 'comdlg32' name 'ReplaceTextA';
function ChooseFont(_para1:LPCHOOSEFONT):WINBOOL; stdcall; external 'comdlg32' name 'ChooseFontA';
function PrintDlg(_para1:LPPRINTDLG):WINBOOL; stdcall; external 'comdlg32' name 'PrintDlgA';
function PageSetupDlg(_para1:LPPAGESETUPDLG):WINBOOL; stdcall; external 'comdlg32' name 'PageSetupDlgA';
{$else}
function GetOpenFileName(_para1:LPOPENFILENAME):WINBOOL; stdcall; external 'comdlg32' name 'GetOpenFileNameW';
function GetSaveFileName(_para1:LPOPENFILENAME):WINBOOL; stdcall; external 'comdlg32' name 'GetSaveFileNameW';
function GetFileTitle(_para1:LPCWSTR; _para2:LPWSTR; _para3:WORD):integer; stdcall; external 'comdlg32' name 'GetFileTitleW';
function ChooseColor(_para1:LPCHOOSECOLOR):WINBOOL; stdcall; external 'comdlg32' name 'ChooseColorW';
function ReplaceText(_para1:LPFINDREPLACE):HWND; stdcall; external 'comdlg32' name 'ReplaceTextW';
function ChooseFont(_para1:LPCHOOSEFONT):WINBOOL; stdcall; external 'comdlg32' name 'ChooseFontW';
function FindText(_para1:LPFINDREPLACE):HWND; stdcall; external 'comdlg32' name 'FindTextW';
function PrintDlg(_para1:LPPRINTDLG):WINBOOL; stdcall; external 'comdlg32' name 'PrintDlgW';
function PageSetupDlg(_para1:LPPAGESETUPDLG):WINBOOL; stdcall; external 'comdlg32' name 'PageSetupDlgW';
{$endif}

function GetOpenFileNameA(_para1:LPOPENFILENAME):WINBOOL; stdcall; external 'comdlg32' name 'GetOpenFileNameA';
function GetSaveFileNameA(_para1:LPOPENFILENAME):WINBOOL; stdcall; external 'comdlg32' name 'GetSaveFileNameA';
function GetFileTitleA(_para1:LPCSTR; _para2:LPSTR; _para3:WORD):integer; stdcall; external 'comdlg32' name 'GetFileTitleA';
function ChooseColorA(_para1:LPCHOOSECOLOR):WINBOOL; stdcall; external 'comdlg32' name 'ChooseColorA';
function FindTextA(_para1:LPFINDREPLACE):HWND; stdcall; external 'comdlg32' name 'FindTextA';
function ReplaceTextA(_para1:LPFINDREPLACE):HWND; stdcall; external 'comdlg32' name 'ReplaceTextA';
function ChooseFontA(_para1:LPCHOOSEFONT):WINBOOL; stdcall; external 'comdlg32' name 'ChooseFontA';
function PrintDlgA(_para1:LPPRINTDLG):WINBOOL; stdcall; external 'comdlg32' name 'PrintDlgA';
function PageSetupDlgA(_para1:LPPAGESETUPDLG):WINBOOL; stdcall; external 'comdlg32' name 'PageSetupDlgA';
function CommDlgExtendedError : DWORD; stdcall; external 'comdlg32' name 'CommDlgExtendedError';
function GetOpenFileNameW(_para1:LPOPENFILENAME):WINBOOL; stdcall; external 'comdlg32' name 'GetOpenFileNameW';
function GetSaveFileNameW(_para1:LPOPENFILENAME):WINBOOL; stdcall; external 'comdlg32' name 'GetSaveFileNameW';
function GetFileTitleW(_para1:LPCWSTR; _para2:LPWSTR; _para3:WORD):integer; stdcall; external 'comdlg32' name 'GetFileTitleW';
function ChooseColorW(_para1:LPCHOOSECOLOR):WINBOOL; stdcall; external 'comdlg32' name 'ChooseColorW';
function ReplaceTextW(_para1:LPFINDREPLACE):HWND; stdcall; external 'comdlg32' name 'ReplaceTextW';
function ChooseFontW(_para1:LPCHOOSEFONT):WINBOOL; stdcall; external 'comdlg32' name 'ChooseFontW';
function FindTextW(_para1:LPFINDREPLACE):HWND; stdcall; external 'comdlg32' name 'FindTextW';
function PrintDlgW(_para1:LPPRINTDLG):WINBOOL; stdcall; external 'comdlg32' name 'PrintDlgW';
function PageSetupDlgW(_para1:LPPAGESETUPDLG):WINBOOL; stdcall; external 'comdlg32' name 'PageSetupDlgW';

implementation

<<<<<<< HEAD
<<<<<<< HEAD
end.
=======
end.
>>>>>>> graemeg/fixes_2_2
=======
end.
>>>>>>> origin/fixes_2_2
