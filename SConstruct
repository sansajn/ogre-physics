# List of pkg-config based library dependencies as (package, version) pair or just package as string (e.g. ('libzmq', '>= 4.3.0') pair or 'libzmq' string).
dependencies = [
	('bullet', '>= 2.88'),
	('OGRE-Bites', '>= 1.12.9'),
	('OGRE-Overlay', '>= 1.12.9'),
	('OGRE-RTShaderSystem', '>= 1.12.9')
]

def build():
	cpp17 = Environment(CCFLAGS=['-std=c++17', '-Wall', '-O0', '-g'])

	cpp17 = configure(cpp17, dependencies)

	cpp17.Program('cube_rain', ['cube_rain.cpp', 'axis.cpp', 'physics.cpp'])


def configure(env, dependency_list):
	conf = env.Configure(
		custom_tests={'CheckPkgVersion': check_pkg_version})

	for dep in dependency_list:
		pkg = ("%s %s" % dep) if type(dep) == tuple else dep
		if not conf.CheckPkgVersion(pkg):
			print("'%s' package not found" % pkg)
			Exit(1)

	conf_env = conf.Finish()

	pkg_conf = 'pkg-config --cflags --libs ' + ' '.join(
		map(lambda dep: dep[0] if type(dep) == tuple else dep, dependencies))

	conf_env.ParseConfig(pkg_conf)

	return conf_env

def check_pkg_version(context, pkg):
	"""custom pkg-config based package version check"""

	context.Message("Checking for '%s' package... " % pkg)
	res = context.TryAction("pkg-config --exists '%s'" % pkg)[0]
	context.Result(res)
	return res

build()
