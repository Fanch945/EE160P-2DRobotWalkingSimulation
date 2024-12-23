model = """<?xml version="1.0"?>
<robot name="five_link_biped">

    <material name="green">
        <color rgba=".3 .6 .4 1"/>
    </material>
    <material name="red">
        <color rgba=".9 .1 0 1"/>
    </material>
    <material name="blue">
        <color rgba="0 0 1 1"/>
    </material>

    <!-- Link definitions -->

    <link name="biped_base">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <mass value="0.000001" />
            <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100" />
        </inertial>
    </link>


    <link name="tibia_left">
        <inertial>
            <origin xyz="0 0 -0.128" rpy="0 0 0" />
            <mass value="3.2" />
            <inertia ixx="0.93" ixy="0" ixz="0" iyy="0.93" iyz="0" izz="0.93" />
        </inertial>
        <visual>
            <origin xyz="0 0 -0.2" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.01" length="0.4" />
            </geometry>
            <material name="green" />
        </visual>
        <collision>
            <origin xyz="0 0 -0.4" rpy="0 0 0" />
            <geometry>
                <!-- <cylinder radius="0.01" length="0.4" /> -->
                <sphere radius="0.01" />
            </geometry>
        </collision>
    </link>

    <link name="tibia_right">
        <inertial>
            <origin xyz="0 0 -0.128" rpy="0 0 0" />
            <mass value="3.2" />
            <inertia ixx="0.93" ixy="0" ixz="0" iyy="0.93" iyz="0" izz="0.93" />
        </inertial>
        <visual>
            <origin xyz="0 0 -0.2" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.01" length="0.4" />
            </geometry>
            <material name="blue" />
        </visual>
        <collision>
            <origin xyz="0 0 -0.4" rpy="0 0 0" />
            <geometry>
                <!-- <cylinder radius="0.01" length="0.4" /> -->
                <sphere radius="0.01" />
            </geometry>
        </collision>
    </link>

    <link name="femur_left">
        <inertial>
            <origin xyz="0 0 -0.163" rpy="0 0 0" />
            <mass value="6.8" />
            <inertia ixx="1.08" ixy="0" ixz="0" iyy="1.08" iyz="0" izz="1.08" />
        </inertial>
        <visual>
            <origin xyz="0 0 -0.2" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.01" length="0.4" />
            </geometry>
            <material name="green" />
        </visual>
        <!-- <collision>
            <origin xyz="0 0 -0.2" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.01" length="0.4" />
            </geometry>
        </collision> -->
    </link>

    <link name="femur_right">
        <inertial>
            <origin xyz="0 0 -0.163" rpy="0 0 0" />
            <mass value="6.8" />
            <inertia ixx="1.08" ixy="0" ixz="0" iyy="1.08" iyz="0" izz="1.08" />
        </inertial>
        <visual>
            <origin xyz="0 0 -0.2" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.01" length="0.4" />
            </geometry>
            <material name="blue" />
        </visual>
        <!-- <collision>
            <origin xyz="0 0 -0.2" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.01" length="0.4" />
            </geometry>
        </collision> -->
    </link>

    <link name="torso">
        <inertial>
            <origin xyz="0 0 0.2" rpy="0 0 0" />
            <mass value="20" />
            <inertia ixx="2.22" ixy="0" ixz="0" iyy="2.22" iyz="0" izz="2.22" />
        </inertial>
        <visual>
            <origin xyz="0 0 0.3125" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.01" length="0.625" />
            </geometry>
            <material name="red" />
        </visual>
        <!-- <collision>
            <origin xyz="0 0 0.3125" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.01" length="0.625" />
            </geometry>
        </collision> -->
    </link>

    <!-- Joint definitions -->

    <joint name="torso_base" type="continuous">
        <parent link="biped_base" />
        <child link="torso" />
        <origin xyz="0 0 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="hip_left" type="continuous">
        <parent link="biped_base" />
        <child link="femur_left" />
        <origin xyz="0 0 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="hip_right" type="continuous">
        <parent link="biped_base" />
        <child link="femur_right" />
        <origin xyz="0 0 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="knee_left" type="continuous">
        <parent link="femur_left" />
        <child link="tibia_left" />
        <origin xyz="0 0 -0.4" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="knee_right" type="continuous">
        <parent link="femur_right" />
        <child link="tibia_right" />
        <origin xyz="0 0 -0.4" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <!-- Transmission definitions -->

    <transmission type="SimpleTransmission" name="torso_base_actuator">
        <actuator name="torso_base_motor" />
        <joint name="torso_base" />
    </transmission>

    <transmission type="SimpleTransmission" name="hip_left_actuator">
        <actuator name="hip_left_motor" />
        <joint name="hip_left" />
    </transmission>

    <transmission type="SimpleTransmission" name="knee_left_actuator">
        <actuator name="knee_left_motor" />
        <joint name="knee_left" />
    </transmission>

    <transmission type="SimpleTransmission" name="hip_right_actuator">
        <actuator name="hip_right_motor" />
        <joint name="hip_right" />
    </transmission>

    <transmission type="SimpleTransmission" name="knee_right_actuator">
        <actuator name="knee_right_motor" />
        <joint name="knee_right" />
    </transmission>

</robot>
"""