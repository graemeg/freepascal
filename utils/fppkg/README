This is the FPC packager.

The architecture is as follows:

A mirror list is maintained with FPC repositories.
The packager will download the mirror list and store
it somewhere locally

From a mirror, a repository is chosen (using it's name)

From the repository the repository file is downloaded.
It contains all known packages. The fprepos.pp unit contains the
functionality to read and maintain the package file.

The packager maintains a local repository file. when needed
it will download a package and install it. It does so recursively.

Each package contains a fpmake.pp driver. It contains all that
is needed to compile, install and zip a package. It can also
create a manifest file (in XML) to import in a repository.

All packager functionality will be implemented in units so
these units can be used in command-line and GUI package managers.

All packager command handling is implemented in descendents fom
TPackageHandler (pkghandler.pp). All messages emitted by these
handlers are in pkgmessages.

Units:
-----

fprepos:
  A unit that describes a repository.
  It is mainly a collection of packages.
fpxmlrep
  A unit to read/write a repository to XML. It can be used to read a
  manifest file as produced by the fpmake driver.
pkgropts
  A unit to maintain settings for the packager it reads/writes
  options from/to an ini file.
fpmkunit
  this is the unit that implements all the functionality a fpmake driver
  needs.
fpmktype
  types and constants shared by fpmkunit and fprepos
rep2xml
  test program for the fprepos unit.
reptest
  creates some test packages for use in the rep2xml test program.
fppkg
  the package manager application
fpmkconv
  Makefile.fpc to fpmake converter.
pkgmessages
  the messages used in pkghandler and below.
pkghandler
  the base for all fppkg command handlers
pkgdownload
  a base implementation for download functionality in fppkg
pkgwget
  a downloader based on spawning wget.
pkgsynapse
  a downloader based on Synapse. Do not put in makefile, as Synapse is
  not distributed by default with FPC.
pkglibcurl
  a downloader based on LibCurl (plain C library).
pkgocurl
  a downloader based on CurlPas (object version). Do not put in makefile,
  as CurlPas is not distributed with FPC.
pkglnet
  a downloader based on lNet. The library is distributed in "lnet" subdir
  of fppkg root.


Options supported in packager config file:
------------------------------------------

LocalMirrors
  Local file with list of mirrors.
RemoteMirrors
  URL with location of remote mirrors file.
RemoteRepository
  Name of repository as found in LocalMirrors file.
LocalRepository
  Location of local repository file.
InstallDir
  Directory to install packages/units to
BuildDir
  Directory where to unpack and build packages
Compiler
  Compiler binary to be used
OS
  Default OS
CPU
  Default CPU

Defaults can be found in pkgropts


Helpfull commands for building packages:
----------------------------------------

* Generate AddInclude and AddUnit lines from an existing PPU file:

ppudump <unit> | awk "/^Source/ { printf(\"AddInclude('%s');\\n\",\$5); } /^Uses unit/ { printf(\"AddUnit('%s');\\n\",tolower(\$3)); }"

* Testing if building a package from archive works:

fpc fpmake && fpmake archive && fppkg build *.zip

* Comparing units directories and generate AddUnit lines for missing .ppu files in <newunitdir>:

diff -q <oldunitdir> <newunitdir> | awk "/^Only.*ppu/ { gsub(\".ppu\",\".pp\"); printf(\"T:=P.Targets.AddUnit('%s');\n\",\$NF); }"
