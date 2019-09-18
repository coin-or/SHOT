# Instructions for compiling SHOT

## Compiling SHOT for Windows using mingw on Linux

1. Install mingw32 (x86_64-w64). 
2. Clone the CoinBrew script and use it to download Cbc. Then build using the following command.
    `./coinbrew build Cbc --prefix=/opt/mingw --target=x86_64-w64-mingw32 --host=x86_64-w64-mingw32 --disable-shared --enable-static=yes --prefix=/opt/mingw --verbosity=4`
3. Install the binary and libraries `sudo coinbrew install Cbc`.
4. Use the CoinBrew script to download Ipopt. Then build using the following command.
    `./coinbrew build Cbc --prefix=/opt/mingw --target=x86_64-w64-mingw32 --host=x86_64-w64-mingw32 --disable-shared --prefix=/opt/mingw --verbosity=4 ADD_FFLAGS="-fopenmp -static-libstdc++ -static-libgcc -static-libgfortran" --enable-threads=posix`.
5. Install the binary and libraries `sudo coinbrew install Ipopt`.
6. Clone the SHOT repository, and create and cd to a build folder `build`.  
7. You might need to edit the `CMakeLists.txt` file to set the paths etc. 
8. Create the CMake build scripts `-DCMAKE_TOOLCHAIN_FILE=../misc/toolchain-mingw.cmake ../`.
9. Compile with `make`.
10. You will need to copy some additional libraries for the dependencies from the `/usr/` sub folders. These are: