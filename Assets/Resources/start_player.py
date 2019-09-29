import os
import sys
import subprocess
import platform

build_directory = os.path.dirname(os.path.realpath(sys.argv[0]))

if platform.system() == 'Windows':
    library_path = build_directory + '\\RosApplication_Data\\Plugins'
    if not (library_path in os.environ['PATH']):
        os.environ['PATH'] = os.environ['PATH'] + ';' + library_path
else:
    library_path = build_directory + '/RosApplication_Data/Plugins'
    if 'LD_LIBRARY_PATH' in os.environ:
        os.environ['LD_LIBRARY_PATH'] = os.environ['LD_LIBRARY_PATH'] + ':' + library_path
    else:
        os.environ['LD_LIBRARY_PATH'] = library_path


if platform.system() == 'Windows':
    subprocess.call(['RosApplication.exe'])
else:
    player_directory = os.path.dirname(os.path.realpath(sys.argv[0]))
    player_path = player_directory + '/RosApplication'
    print("Binary path: " + player_path)
    subprocess.call([player_path])
