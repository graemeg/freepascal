{

    Basic Servlet Support
    Copyright (c) 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}{$H+}
unit Servlets;

interface

uses SysUtils, Classes;

type

  EServlet = class(Exception);

  TServletContext = class
  public
    property Attributes[const AName: String]: TObject;  // !!!: Implement this  rw
    // function GetContext(const URIPath: String): TServletContext;     // !!!: How to implement?
    // function GetRealPath(const APath: String): String;       // !!!: How to implement?
    property ServletContextName: String;        // !!!: How to implement?
    // procedure Log(const AMsg: String);       // !!!: Implement this
    // procedure RemoveAttribute(const AName: String);  // !!!: Implement this
  end;

  TServletRequest = class
  private
    FInputStream: TStream;
    FScheme, FPathInfo: String;
  protected
    function GetContentLength: Integer; virtual; abstract;
    function GetContentType: String; virtual; abstract;
    function GetProtocol: String; virtual; abstract;
  public
    constructor Create(AInputStream: TStream; const AScheme, APathInfo: String);
    property Attributes[const AName: String]: TObject;  // !!!: Implement this  rw
    property CharacterEncoding: String; // !!!: Implement this  rw
    property ContentLength: Integer read GetContentLength;
    property ContentType: String read GetContentType;
    property InputStream: TStream read FInputStream;
    property Parameters[const AName: String]: String;   // !!!: Implement this
    property ParameterValues[const AName: String]: TStrings;    // !!!: Implement this
    property Protocol: String read GetProtocol;
    property RemoteAddr: String;        // !!!: Implement this
    property RemoteHost: String;        // !!!: Implement this
    property Scheme: String read FScheme;
    property ServerName: String;        // !!!: How to implement?
    property ServerPort: Integer;       // !!!: How to implement?
    property IsSecure: Boolean;         // !!!: Implement this

    // procedure RemoveAttribute(const AName: String);  // !!!: Implement this
  end;

  TServletResponse = class
  private
    FOutputStream: TStream;
  protected
    procedure SetContentType(const Value: String); virtual; abstract;
    procedure SetContentLength(Value: Int64); virtual; abstract;
  public
    constructor Create(AOutputStream: TStream);
    property BufferSize: Integer;       // !!!: How to implement?  rw
    property CharacterEncoding: String; // !!!: Implement this
    property ContentLength: Int64 write SetContentLength;
    property ContentType: String write SetContentType;
    property OutputStream: TStream read FOutputStream;
    property IsCommitted: Boolean;      // !!!: Implement this

    // procedure FlushBuffer;           // !!!: Implement this
    // procedure Reset;                 // !!!: Implement this
    // procedure ResetBuffer;           // !!!: Implement this
  end;

  TGenericServlet = class(TComponent)
  public
    procedure Service(Req: TServletRequest; Resp: TServletResponse);
      virtual; abstract;
    property ServletContext: TServletContext;   // !!!: Implement this
  end;



implementation


constructor TServletRequest.Create(AInputStream: TStream;
  const AScheme, APathInfo: String);
begin
  inherited Create;
  FInputStream := AInputStream;
  FScheme := AScheme;
  FPathInfo := APathInfo;
end;


constructor TServletResponse.Create(AOutputStream: TStream);
begin
  inherited Create;
  FOutputStream := AOutputStream;
end;


end.
