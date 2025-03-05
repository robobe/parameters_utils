

# Build

```bash
colcon build --packages-select parameters_manager_ex \
    --cmake-args -DCMAKE_TOOLCHAIN_FILE=`pwd`/cmake/toolchain.cmake \
    -DINCLUDE_CPACK=`pwd`/cmake/Packing.cmake \
    -DOUTPUT_FOLDER=`pwd`/debs \
    -DARCH=aarch64
```

```bash
colcon build --packages-select g_stream_interface \
    --cmake-args -DCMAKE_TOOLCHAIN_FILE=`pwd`/cmake/toolchain.cmake \
    -DINCLUDE_CPACK=`pwd`/cmake/Packing.cmake \
    -DOUTPUT_FOLDER=`pwd`/debs \
    -DARCH=aarch64
```

```bash
colcon build --packages-select g_stream_control \
    --cmake-args -DCMAKE_TOOLCHAIN_FILE=`pwd`/cmake/toolchain.cmake \
    -DINCLUDE_CPACK=`pwd`/cmake/Packing.cmake \
    -DOUTPUT_FOLDER=`pwd`/debs \
    -DARCH=aarch64
```

```bash
colcon build --packages-select g_stream \
    --cmake-args -DCMAKE_TOOLCHAIN_FILE=`pwd`/cmake/toolchain.cmake \
    -DINCLUDE_CPACK=`pwd`/cmake/Packing.cmake \
    -DOUTPUT_FOLDER=`pwd`/debs \
    -DARCH=aarch64
```