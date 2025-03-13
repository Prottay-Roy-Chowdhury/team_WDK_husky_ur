# WDK - ur


## Step 1 - Build the image

````
.docker/build_image.sh
````

## Step 2 - Run the image

````
.docker/run_user_nvidia.sh
````

## Step 3 - Give access to the workspace

````
sudo chown -R <your_username> /dev_ws
````

## Step 4 - Launch ur_commander

````
ros2 launch ur_commander iaac_ur10e.launch.py sim:=true
````

## Step 5 - Launch static_transform

````
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 ur_base_link world
````

## Step 6 - Launch husky_visuallization

````
ros2 launch clearpath_viz  view_model.launch.py setup_path:=/dev_ws/src/husky_commander/config/husky_basic/
````

## Step 7 - Run the example Jupyter notebook in dev_ws/src/ur_commander/scripts/ur_controller.ipynb

Now enjoy and have fun:)

