{
    $Id: header,v 1.1 2000/07/13 06:33:45 michael Exp $
    This file is part of the Free Pascal packages.
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$H+}
program testcurl;

uses libcurl;

Var 
  URL : Pchar = 'http://www.freepascal.org';
  hCurl : pCurl;

begin
  hCurl:= curl_easy_init;
  if Assigned(hCurl) then
    begin
    curl_easy_setopt(hCurl,CURLOPT_VERBOSE, [True]);
    curl_easy_setopt(hCurl,CURLOPT_URL,[URL]);
    curl_easy_perform(hCurl);
    curl_easy_cleanup(hCurl);
    end;
end.
