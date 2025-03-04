# Parameters Manager Ex

Allow external parameter file to be load into node context
It's expose new service that SAVE the parameters that load from the file
The parameters the load / insert by the manager behaved like regular node parameter


```bash
colcon build --packages-select parameters_manager_ex \
    --symlink-install false \
    --cmake-args -DCMAKE_TOOLCHAIN_FILE=`pwd`/src/g_stream_interface/cmake/toolchain.cmake \
    -DINCLUDE_CPACK=`pwd`/submodules/parameters_utils/src/parameters_manager_ex/cmake/Packing.cmake \
    -DOUTPUT_FOLDER=/tmp \
    -DARCH=aarch64
```