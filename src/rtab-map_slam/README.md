# RTAB-Map SLAM wrapper for MiniROS
**Before installing actual MiniROS package you
need to install the rtabmap_py submodule manually.
To do this:**
- install required packages: 
```
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libsqlite3-dev \
    libpcl-dev \
    libopencv-dev \
    libproj-dev \
    libqt5svg5-dev \
    pkg-config \
    gdb \
	ninja-build
```

- clone RTAB-Map repository: 
```
git clone https://github.com/introlab/rtabmap.git
cd rtabmap
```

- compile RTAB-Map from source and install:
```
mkdir build
cd build
cmake -G Ninja ..
ninja
sudo ninja install
sudo ldconfig
```

- run:
```
cd rtabmap_py
pip install .
```