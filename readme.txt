This is a BMP library designed by Thibault MARECHAL

Installation
--------------------------------------------------------------------------------

To install this library, just place this entire folder as a subfolder in your
Arduino/lib/targets/libraries folder.

When installed, this library should look like:

Arduino/lib/targets/libraries/BMP180_TML 		             (this library's folder)
Arduino/lib/targets/libraries/BMP180_TML/src/BMP180_TML.cpp  (the library implementation file)
Arduino/lib/targets/libraries/BMP180_TML/src/BMP180_TML.h 	 (the library description file)
Arduino/lib/targets/libraries/BMP180_TML/keywords.txt 		 (the syntax coloring file)
Arduino/lib/targets/libraries/BMP180_TML/examples     		 (the examples in the "open" menu)
Arduino/lib/targets/libraries/BMP180_TML/readme.txt   		 (this file)

Building
--------------------------------------------------------------------------------

After this library is installed, you just have to start the Arduino application.
You may see a few warning messages as it's built.

To use this library in a sketch, go to the Sketch | Import Library menu and
select BMP180_TML.  This will add a corresponding line to the top of your sketch:
#include <BMP180_TML.h>

To stop using this library, delete that line from your sketch.

Geeky information:
After a successful build of this library, a new file named "BMP180_TML.o" will appear
in "Arduino/lib/targets/libraries/BMP180_TML". This file is the built/compiled library
code.

If you choose to modify the code for this library (i.e. "BMP180_TML.cpp" or "BMP180_TML.h"),
then you must first 'unbuild' this library by deleting the "BMP180_TML.o" file. The
new "BMP180_TML.o" with your code will appear after the next press of "verify"

