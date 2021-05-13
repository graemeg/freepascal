# Fcl-web integration tests

This is an attempt to test the data-processing inside fcl-web.

There are two parts: A fpc-application (fcgi_dump_request) that returns the
received http-data in a Json-format. And an Apache jMeter test-script.

The open-source tool jMeter form Apache is used to send http-requests to the
fpc-application, and to do several assertions on the results.

To run, you have to create a setup that supports the fcgi_dump_request
application. For example using a Apache or NGINX setup.

Then check the local-settings within the jMeter test script, and run it.

In principle it is also possible to test plain CGI or the embeddes web-server
with small changes to fcgi_dump_request.pp.

Note that at the end of the jMeter script a call is made that makes the
fcgi_dump_request terminate. When compiled with heap-trace information
this can help locating memory leaks
