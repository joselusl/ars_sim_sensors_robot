<!-- -->
<launch>

	<arg name="ars_sim_sensor_pos_robot_node_name" 	default="ars_sim_sensor_pos_robot_node" />
	<arg name="screen" 	default="screen" />


  <node name="$(arg ars_sim_sensor_pos_robot_node_name)" pkg="ars_sim_sensors_robot" type="ars_sim_sensor_atti_robot_ros_node.py" output="$(arg screen)" >


  </node>


</launch>
