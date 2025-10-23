# Lightly Roasted LiPo's Warehouse Bot
## Generate Map
1. Run slam toolbox on the turtlebot to start building up the map; `ros2 launch mtrx3760_lrl_warehousebot online_async_launch.py`
2. Drive around map until the borders are nice and thick, going for more passes generally do the trick.
3. Save the map data to the map folder; `cd ~/turtlebot3_ws; ros2 run nav2_map_server map_saver_cli -f maps/<map_name>`

## Run AMCL
1. Launch the map server, and point it to the saved map; `cd ~/turtlebot3_ws; ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=maps/my_map.yaml`
2. Launch the amcl server; `ros2 launch mtrx3760_lrl_warehousebot solo_amcl_launch.py`
3. Setup Rviz, and subscribe it to /map, set quality of service to **Best Effort** for /map and /map_updates
4. Configure the map server; `ros2 lifecycle set /map_server configure`
5. Activate the map server; `ros2 lifecycle set /map_server activate`
6. Double check the map has shown up in Rviz, if not deactivate and active the map server again.
7. Configure the amcl node; `ros2 lifecycle set /amcl configure`
8. Use Rviz to feed amcl a pose estimate.
9. Activate the amcl node; `ros2 lifecycle set /amcl activate`
