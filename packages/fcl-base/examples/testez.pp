
program CGITest;

{$mode delphi}

uses classes, ezcgi;

// In the following class you only need to use either DoPost or DoGet
// depending on what type of CGI request you want to use. If you only
// are going to be making GET requests that you only need to define
// and override DoGet and like wise for POST requests.

type
   TCGIData = class(TEZcgi)
      procedure DoPost; override;
      procedure DoGet; override;
      procedure ShowStuff;
   end;

var
   cgiStuff : TCGIData;

// All output from the CGI must first begin with the WriteContent
// call. This places the passed information and the correct CR's
// required. In most cases you will always use it as shown below.
// However if you ever need to return content that isn't standard
// text/html this allows you to set what that content type is.
//
// The PutLine call just passes the information indicated out into
// the stream for return to the client browser.


procedure TCGIData.DoGet;
begin
   WriteContent('text/html');
   PutLine('++++++++++ Using GET +++++++++');
   ShowStuff;
end;

procedure TCGIData.DoPost;
begin
   WriteContent('text/html');
   PutLine('++++++++++ Using POST +++++++++');
   ShowStuff;
end;

// I wrote this method so that you can see everything that is passed to
// and stored by the TEZcgi class. It currently stores all normal CGI
// as well as everything passed to the CGI by the client. These
// variables can be accessed via the following properties.
//
// Values[Index : string];
// The Index data can be something like AUTH_TYPE, SCRIPT_NAME, or any
// standard HTML or CGI environment variable. This can also be the
// field name of the content being passed in a query string such as
// "name" found in  //URL/CGI/cgiapp?name=bob
//
// Another routine is availble GetValue(Index, defaultValue : string);
// This routine does the same as the property Values except it allows
// you to set a default value to be returned if no variable of type
// name index is found.

// This data is stored in a TStringList so you can retreive it by index
// as well if you know the index location of the information.
//
// The properties for doing this are Names[index : integer] and
// Variables[index : integer]. Names returns the name of the variable,
// this would be the data passed in the values or GetValue calls.
// Instead of returning the value "bob" it returns the value "name". The
// variables property returns the whole string so using the name example
// above it would return "name=bob".
//
// To determine how many environment variables have been stored you can
// use the VariableCount property.
//
// The following procedure loops through all of the enviroment variables
// and prints them back to the client. This is a good CGI example to
// show you exactly what information you have to work with.

procedure TCGIData.ShowStuff;
var
   loop : integer;

begin
   for loop := 0 to VariableCount - 1 do
      PutLine(Variables[loop] + '<BR>');
end;

// what follows is the actual program begin..end.
// The Create call for the class does all the work of the CGI loading
// the environment variables.
// The FName and FEmail data is used by the Error report. If the CGI
// doesn't work correctly for the user this data is included in an
// error report that is returned to the user which tells them who to
// inform and how.
// The Run command starts processing the CGI request and eventually
// calls either the DoGet or DoPost depending on the request type.

// I don't try and trap it but the class does raise an exception called
// ECGIException when some error is generated.

begin
   try
      cgiStuff := TCGIData.Create;
      cgiStuff.Name := 'Michael A. Hess';    // replace with your name
      cgiStuff.Email := 'mhess@miraclec.com';// replace with your email
      cgiStuff.Run;
   finally
      cgiStuff.Free;
   end;
end.
