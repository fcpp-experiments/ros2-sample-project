# Install 

```bash
rosdep install --from-paths src --ignore-src -r -y -i

# Import distribution dependencies
colcon build --event-handlers console_direct+ --symlink-install
```
