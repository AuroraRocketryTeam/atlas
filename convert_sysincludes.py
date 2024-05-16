from pathlib import Path
Import("env" ,"projenv")

platform = env.PioPlatform()
FRAMEWORK_DIR = Path(platform.get_package_dir("framework-arduinoespressif32"))
# apply these changes to current working env, the project env and the global env
for e in (env, projenv, DefaultEnvironment()):
    framework_includes = list()
    filtered_cpppath = list()
    for p in e["CPPPATH"]:
        # is the current include path inside the framework directory?
        if FRAMEWORK_DIR in Path(p).parents:
            framework_includes.append(p)
        else:
            filtered_cpppath.append(p)
    e.Replace(CPPPATH=filtered_cpppath)
    e.Append(CCFLAGS=[("-isystem", p) for p in framework_includes])