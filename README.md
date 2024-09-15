# CSM-LANCE-Dev
Personal development workspace for code used on the LANCE serices of CSM robots.

## Build
* Install rosdep dependencies
```bash
sudo rosdep init
rosdep update
rosdep install --ignore-src --from-paths . -r -y
```
* Check [submodule readme](./temp_perception/README.md) for dependency install instructions (GTSAM)
* Build
```
colcon build --symlink-install [--event-handlers console_direct+] [--executor parallel] [--cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON]
source install/setup.bash
```

__*Last updated: 9/14/24*__