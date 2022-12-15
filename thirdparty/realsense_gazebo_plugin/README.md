# Intel RealSense Gazebo ROS plugin

This package is a Gazebo ROS plugin for the Intel D435 realsense camera.

## Note
This branch is aimed for ROS2, if you are ROS1 user you can see the other branches(e.g melodic)
 
## Acknowledgement

This is a continuation of work done by [SyrianSpock](https://github.com/SyrianSpock) for a Gazebo ROS plugin with RS200 camera.

This package also includes the work developed by Intel Corporation with the ROS model for the [D435](https://github.com/intel-ros/realsense) camera.

## Example usage with a custom robot

Note that this was tested for the ROS2 branch with ROS Foxy distro. A turtlebot3 like custom robot model was used. 
In custom robot's `model.sdf`, we should attach the link, sensors, joint  and plugin block as following; 

```xml
    <link name="realsense_link">
      <pose>0.4 0 0.25 0 0 0</pose>
      <visual name="realsense_link_visual">
        <pose>0 0 0 -1.57 0 -1.57</pose>
        <geometry>
          <mesh>
            <uri>model://chiconybot/meshes/d435.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="realsense_link_collision">
        <pose>0 0 0 -1.57 0 -1.57</pose>
        <geometry>
          <box>
            <size>0.02505 0.090 0.025</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.001</iyy>
          <iyz>0.000</iyz>
          <izz>0.001</izz>
        </inertia>
        <mass>0.564</mass>
      </inertial>

      <sensor name="cameradepth" type="depth">
        <camera name="camera">
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.100</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>0</visualize>
      </sensor>
      <sensor name="cameracolor" type="camera">
        <camera name="camera">
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>RGB_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>1</visualize>
      </sensor>
      <sensor name="cameraired1" type="camera">
        <camera name="camera">
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>L_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>1</update_rate>
        <visualize>0</visualize>
      </sensor>
      <sensor name="cameraired2" type="camera">
        <camera name="camera">
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>L_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>1</update_rate>
        <visualize>0</visualize>
      </sensor>
    </link>

    <joint name="realsense_joint" type="fixed">
      <parent>base_link</parent>
      <child>realsense_link</child>
      <pose>0.4 0 0.4 0 0 0</pose>
    </joint>

    <plugin name="camera" filename="librealsense_gazebo_plugin.so">
      <prefix>camera</prefix>
      <depthUpdateRate>30.0</depthUpdateRate>
      <colorUpdateRate>30.0</colorUpdateRate>
      <infraredUpdateRate>1.0</infraredUpdateRate>
      <depthTopicName>aligned_depth_to_color/image_raw</depthTopicName>
      <depthCameraInfoTopicName>depth/camera_info</depthCameraInfoTopicName>
      <colorTopicName>color/image_raw</colorTopicName>
      <colorCameraInfoTopicName>color/camera_info</colorCameraInfoTopicName>
      <infrared1TopicName>infra1/image_raw</infrared1TopicName>
      <infrared1CameraInfoTopicName>infra1/camera_info</infrared1CameraInfoTopicName>
      <infrared2TopicName>infra2/image_raw</infrared2TopicName>
      <infrared2CameraInfoTopicName>infra2/camera_info</infrared2CameraInfoTopicName>
      <colorOpticalframeName>camera_color_optical_frame</colorOpticalframeName>
      <depthOpticalframeName>camera_depth_optical_frame</depthOpticalframeName>
      <infrared1OpticalframeName>camera_left_ir_optical_frame</infrared1OpticalframeName>
      <infrared2OpticalframeName>camera_right_ir_optical_frame</infrared2OpticalframeName>
      <rangeMinDepth>0.3</rangeMinDepth>
      <rangeMaxDepth>3.0</rangeMaxDepth>
      <pointCloud>true</pointCloud>
      <pointCloudTopicName>depth/color/points</pointCloudTopicName>
      <pointCloudCutoff>0.3</pointCloudCutoff>
    </plugin>
```

Finally we should define the joint, links of each camera(color, depth, ir_right, ir_left) W.R.T robot body, 
In URDF(usually in `xxx_description` package) of the robot add following; 

```xml
  <link name="camera_bottom_screw_frame">
    <visual>
      <geometry>
        <mesh filename="package://chiconybot_description/meshes/sensors/d435.dae" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://chiconybot_description/meshes/sensors/d435.dae" />
      </geometry>
    </collision>
  </link>

  <link name="camera_link"></link>

  <link name="camera_depth_frame"></link>

  <link name="camera_depth_optical_frame"></link>

  <link name="camera_color_frame"></link>

  <link name="camera_color_optical_frame"></link>

  <link name="camera_left_ir_frame"></link>

  <link name="camera_left_ir_optical_frame"></link>

  <link name="camera_right_ir_frame"></link>

  <link name="camera_right_ir_optical_frame"></link>


   <joint name="camera_joint" type="fixed">
    <parent link="base_link" />
    <child link="camera_bottom_screw_frame" />
    <pose xyz="0.4 0 0.25" rpy="0 0 0" />
  </joint>

  <joint name="camera_link_joint" type="fixed">
    <parent link="camera_bottom_screw_frame" />
    <child link="camera_link" />
    <pose xyz="0 0.0175 0.0125 " rpy="0 0 0" />
  </joint>

  <joint name="camera_depth_joint" type="fixed">
    <parent link="camera_link" />
    <child link="camera_depth_frame" />
    <pose xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <joint name="camera_depth_optical_joint" type="fixed">
    <parent link="camera_depth_frame" />
    <child link="camera_depth_optical_frame" />
    <pose xyz="0 0 0 " rpy="-1.57 0 -1.57" />
  </joint>

  <joint name="camera_color_joint" type="fixed">
    <parent link="camera_depth_frame" />
    <child link="camera_color_frame" />
    <pose xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <joint name="camera_color_optical_joint" type="fixed">
    <parent link="camera_color_frame" />
    <child link="camera_color_optical_frame" />
    <pose xyz="0 0 0 " rpy="-1.57 0 -1.57" />
  </joint>

  <joint name="camera_left_ir_joint" type="fixed">
    <parent link="camera_depth_frame" />
    <child link="camera_left_ir_frame" />
    <pose xyz="0 0 0 " rpy="0 0 0 " />
  </joint>

  <joint name="camera_left_ir_optical_joint" type="fixed">
    <parent link="camera_left_ir_frame" />
    <child link="camera_left_ir_optical_frame" />
    <pose xyz="0 0 0 " rpy="-1.57 0 -1.57" />
  </joint>

  <joint name="camera_right_ir_joint" type="fixed">
    <parent link="camera_depth_frame" />
    <child link="camera_right_ir_frame" />
    <pose xyz="0 -0.050 0 " rpy="0 0 0" />
  </joint>

  <joint name="camera_right_ir_optical_joint" type="fixed">
    <parent link="camera_right_ir_frame" />
    <child link="camera_right_ir_optical_frame" />
    <pose xyz="0 0 0 " rpy="-1.57 0 -1.57" />
  </joint>

```
