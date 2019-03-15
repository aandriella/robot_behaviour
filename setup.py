from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['robot_behaviour'],
    #scripts=['actions', 'robot', 'bin/speech_utterance'],
    package_dir={'': 'src'}
)

setup(**d)