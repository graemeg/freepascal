# Testsuite

## Demos

The testsuite can optionally run all demos: define USEDEMOS and fpTestX on the
command-line or in the lazarus defines.

In that case the ../demo and ../demo/polygon directories must be added to
the unit path of the compiler.

You can then run these tests using the following command-line
./testfpreport --suite=TTestDemos

The demo reports will be rendered and saved to a directory "rendered".

The first time you run the demo test, the file will be called demo.set.json.
The second time you run the demo test, if the result differs, the result
will be saved to a file called demo.actual.json.

So, to test changes, first delete all json files in  the rendered directory.
Then do a first run, this will create the initial files, and set a baseline. 
Make your changes, and then run the reports again. You will be notified of
differences.

The reason these files are not stored in SVN is that they are dependent on
the platform:
a) The reports contain newlines. 
   Depending on the platform they will be saved as \r \r\n or \n. 
b) There can and will be localization issues. 

## Fonts 

The demos need some extra fonts. 
The needed fonts (Ubuntu and DejaVu Sans) can be downloaded from:

* https://assets.ubuntu.com/v1/0cef8205-ubuntu-font-family-0.83.zip
* https://www.downloadfonts.io/calibri-font-family-free/
* https://www.fontsquirrel.com/fonts/download/dejavu-sans
* https://www.fontsquirrel.com/fonts/download/liberation-sans

These fonts should be saved to the ./fonts or ../demo/fonts directory.