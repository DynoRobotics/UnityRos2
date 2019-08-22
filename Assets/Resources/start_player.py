import os
import sys
import subprocess

build_directory = os.path.dirname(os.path.realpath(sys.argv[0]))
library_path = build_directory + '\\BuiltGame_Data\\Plugins'

if not (library_path in os.environ['PATH']):
    os.environ['PATH'] = os.environ['PATH'] + ';' + library_path + ';'

subprocess.call(['BuiltGame.exe'])
