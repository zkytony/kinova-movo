## How-tos

### How to start a Gazebo simulation

1. `roslaunch movo_object_search world_bringup.launch`
   
    This command will start the Gazebo simulation (by default, the gui is `true`),
    and other necessary MOVO components. When the Gazebo simulation starts, wait
    for a few seconds until the MOVO dance completes.
    
    By default the environment is `worlds/tabletop_cube.sdf` which is located
    under the `movo_gazebo` package. If you want to change it, edit the default
    value of the parameter `world` in `environment.launch`.
    
### How to start navigation stack

1. `roslaunch movo_object_search navigation.launch`

    This will launch the necessary components for the navigation stack; It
    is just a proxy for `$(find movo_demos)/launch/nav/map_nav.launch` file.
    
    By default the map for the `tabletop_cube` environment is used; The name
    of the map is called `test_simple`, and the `.pgm` and `.yaml` files are
    located under `$(find movo_demos)/map/`. It's very confusing that the world
    models and the map files are not in the same package. But, it is what it is.
    
### How to get volumetric observation?

1. Go to `$(movo_object_search)/scripts/observation` and run `python process_pcl.py`.
   If you want to detect AR tag as well `python process_pcl.py -M`. Note that
   due to ROS-related reasons, `python process_pcl.py -M` will only broadcast
   observations if an AR tag is detected. Therefore, I recommend running these
   two together separately, each with a different node name.
   
### Which RVIZ to use?

1. Use the `nav_cloud.rviz` which allows navigation and also visualizes the observation and point cloud.
