FPKPascal XForms support.
-------------------------

To compile the units in this directory, proceed as follows:

0. You need an installed forms library. You need to know where it is
   installed. Either version 0.86 or 0.88 will do. For version 0.88
   you need version 0.99.8 or higher of the compiler.

2. Edit the Makefile in the current directory . You have to specify:
   - which version of the forms library you use (0.86 or 0.88)
   - The place of the forms library. (if not in /usr/lib).
   - The place of the X library. (should be /usr/X11/lib)
   - Where you want the forms libraries installed.
   - Where you want the fd2pascal filter installed.
   - Optionally, the path to the compiler and the pascal units.

3. You should be all set to compile. type 
    make all 
   This will compile all the needed units, and the fd2pascal program which
   translates .fd design files to pascal programs.
   As of version 0.88, the fdesign program can directly issue pascal
   code if you have fd2pascal installed.

4. To compile the demo programs, type (still in the forms directory)
    make demo     
   This will compile all the demo programs in the demo subdirectory.

5. To install the units and the fd2pascal program, type 
    make install
   This will install all units and the program in the directories you 
   specified. They will be created if they don't exist.

More information on all this can be found on 
  http://tfdec1.fys.kuleuven.ac.be/~michael/fpc-linux/forms.html

Enjoy !
Michael Van Canneyt (michael@tfdec1.fys.kuleuven.ac.be)
