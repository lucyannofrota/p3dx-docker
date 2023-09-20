# FROM lucyannofrota/foxy:zed as a

FROM build:zed as foxy-zed

ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

RUN git clone --branch 1.0.2 https://github.com/lucyannofrota/smap_sampler.git /workspace/src/smap/smap_sampler \
    && git clone --branch 1.0.3 https://github.com/lucyannofrota/smap_interfaces.git /workspace/src/smap/smap_interfaces \
    && cd /workspace \
    && /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
    colcon build --parallel-workers $(nproc) --symlink-install \
    --event-handlers console_direct+ --base-paths src \
    --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
    ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
    ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
    ' --no-warn-unused-cli' "

