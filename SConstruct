cpp17 = Environment(
	CCFLAGS=['-std=c++17', '-Wall', '-O0', '-g'])

cpp17.ParseConfig('pkg-config --libs --cflags OGRE-Bites OGRE-Overlay OGRE-RTShaderSystem')

cpp17.Program('cube_rain', ['cube_rain.cpp', 'axis.cpp'])
