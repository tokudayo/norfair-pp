from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

__version__ = "0.0.1"

ext_modules = [
    Pybind11Extension("norfair_pp",
        ["pybinding.cpp"],
        # Example: passing in the version to the compiled code
        define_macros = [('VERSION_INFO', __version__)],
        ),
]

setup(
    name="norfair_pp",
    version=__version__,
    author="To Duc",
    author_email="20toduc01@gmail.com",
    url="https://github.com/20toduc01/norfair-pp",
    description="Multi-object tracker using Kalman filter. 1 (x, y) point per object.",
    long_description="",
    ext_modules=ext_modules,
    extras_require={},
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.6",
)