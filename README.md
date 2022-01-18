# Norfair++
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/123NTUk6jwIJgf_VHJXC4-0RQWATET5xJ?usp=sharing)

This is a C++ implementation of [Techainer's Norfair](https://github.com/Techainer/norfair), which originates from [Norfair](https://github.com/tryolabs/norfair), a library for real-time 2D object tracking.


Its function is identical to Techainer's version:

> It assigns a track id to each object instead of returning a list of new objects after tracking.
>
> This is optimized for the use case when there is 1 representative point per detection.

This contains some more optimizations in tracker update functions and the use of Kalman filter. Overall, the Python binding for this C++ implementation offers a ~10x speedup compared to Techainer's fork, which was already much faster than the original Norfair (for the above use case).

## Installation
Clone this repository:
```shell
git clone https://github.com/20toduc01/norfair-pp.git
cd norfair-pp
```

This project requires [Eigen3](https://eigen.tuxfamily.org):
```shell
curl https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip -o eigen-3.4.0.zip
unzip eigen-3.4.0
cp -r eigen-3.4.0/Eigen ./norfair_pp/Eigen
rm -rf eigen-3.4.0*
```

## Python binding
This project was built with Python in mind. To install the Python binding, first install [pybind11](https://github.com/pybind/pybind11):
```shell
pip3 install pybind11
```

Then run `setup.py`:
```shell
python3 setup.py install
```

## Usage and demo
Check out [this Colab notebook](https://colab.research.google.com/drive/123NTUk6jwIJgf_VHJXC4-0RQWATET5xJ?usp=sharing) and [the result video](https://www.youtube.com/watch?v=GPeYwYejRUQ).


<a href="http://www.youtube.com/watch?feature=player_embedded&v=GPeYwYejRUQ" target="_blank">
 <img src="http://img.youtube.com/vi/GPeYwYejRUQ/mqdefault.jpg" alt="Watch the video" width="427" height="240" border="30" />
</a>
