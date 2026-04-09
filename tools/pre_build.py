Import('env')

def before_build(source, target, env):
    from SCons.Script import COMMAND_LINE_TARGETS
    import sys

    try:
        print("== Pre-build script starting ==")
        print("Generating compilation database...")

        result = env.Execute("$PYTHONEXE -m platformio run -t compiledb")
        if result != 0:
            print("Failed to generate compilation database")
            sys.exit(1)

    except Exception as e:
        print(f"Pre-build script failed: {str(e)}")
        sys.exit(1)

env.AlwaysBuild(env.Alias('pre_build'))
env.AddPreAction(["$BUILD_DIR/${PROGNAME}.elf", "buildprog"], before_build)
