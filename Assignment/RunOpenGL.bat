REM
echo "Running \m"
gcc -c -o Test1.o TriangleScan_Base.cpp -I\"C:\Compiler\mingw64\x86_64-w64-mingw32\include"
cls
echo "Compiling Graphic"
gcc -o Test1.exe  Test1.o -L\"C:/Compiler/mingw64/x86_64-w64-mingw32/lib -lopengl32 -lglu32 -lfreeglut"
cls
echo "App Running"
Test1.exe