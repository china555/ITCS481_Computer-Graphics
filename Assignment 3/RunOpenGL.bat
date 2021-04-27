REM
echo "Running \m"
gcc -c -o Test1.o plane2_base_a.cpp -I"C:\Compiler\mingw64\x86_64-w64-mingw32\include" -I"C:\Libraries\gmtl-0.6.1"
echo "Compiling Graphic"
g++ -o Test1.exe  Test1.o -L"C:\Compiler\mingw64\x86_64-w64-mingw32\lib" -lopengl32 -lglu32 -lfreeglut
echo "App Running"
Test1.exe