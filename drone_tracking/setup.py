from disutils.core import setup
from catkin_pkg.python_setup import generate_disutils_setup 

d = generate_disutils_setup(
    packages=["drone_tracking"],
    package_dir={"":"src"}
)