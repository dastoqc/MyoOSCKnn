# MyoOSCKnn
OSC broadcast of Myo Knn class

# to install
intall the windows Myo manager "Myo Connect" and pair you device.
recompile with cmake the oscpack :
cmake -G "Visual Studio 10"

install anaconda python 2.7 with all libs and change the include directory for python27.h in the MyoOSCKnn project settings.

Win32 settings may not be up to date, check x64 ones.

# to use
launch the program, press key 1 to 9 until your gesture is done to create a new class, the classifier will be automatically trained.
