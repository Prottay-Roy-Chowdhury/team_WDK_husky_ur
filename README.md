## Connection to husky
```
ssh iaac@10.42.0.1
```
```
tmux new -s husky
```
## How to split tmux terminal
ctrl+b then % or "  
Terminal 1  
```
ros2 launch /etc/clearpath/platform/launch/platform-service.launch.py
```
Terminal 2
```
ros2 launch usb_cam camera.launch.py
```