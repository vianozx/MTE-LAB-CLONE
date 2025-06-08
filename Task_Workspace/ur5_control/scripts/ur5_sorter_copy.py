#! /usr/bin/env python
'''
Modified ur5_sorter.py using standalone mathematical IK module.
Your original lib.py remains unchanged.
'''

import rospy
import geometry_msgs.msg
import os
from env_sim.msg import LogicalCameraImage
from math import radians, degrees
from rospy.exceptions import ROSInterruptException

# Import your original lib (unchanged)
from lib import UR5MoveIt

# Import the new standalone math IK module
from ur5_math_ik import UR5MathIK, calculate_joint_angles

MasterString = ''
ReadyFlag = True

# Create IK solver instance
math_ik = UR5MathIK()

def env_data():
    '''Environment data (unchanged)'''
    box_length = 0.15
    vacuum_gripper_width = 0.117
    return [box_length, vacuum_gripper_width]

def get_robot_positions():
    '''
    Define robot positions in cartesian coordinates.
    Modify these coordinates to change robot behavior.
    '''
    positions = {
        'home': {
            'xyz': [-0.486969, 0.108981, 1.3597412],  # MODIFY: Home position
            'rpy': [1.359741, radians(0), 1.571202],  # Gripper pointing down
        },
        'red_bin': {
            'xyz': [0.11, 1.0, 0.95],  # MODIFY: Red bin position
            'rpy': [0.0, radians(90), 0.0],
        },
        'green_bin': {
            'xyz': [1.0, 0.0, 0.95],  # MODIFY: Green bin position
            'rpy': [0.0, radians(90), radians(-90)],
        },
        'blue_bin': {
            'xyz': [0.11, -1.0, 0.95],  # MODIFY: Blue bin position
            'rpy': [0.0, radians(90), 0.0],
        }
    }
    return positions

def joint_angles_data():
    '''
    Calculate joint angles from coordinates using mathematical IK.
    This replaces your original hardcoded joint angles.
    '''
    positions = get_robot_positions()
    joint_angles_list = []
    
    rospy.loginfo('\033[96m' + "Calculating joint angles using mathematical IK..." + '\033[0m')
    
    position_names = ['home', 'red_bin', 'green_bin', 'blue_bin']
    
    for pos_name in position_names:
        if pos_name in positions:
            pos_data = positions[pos_name]
            x, y, z = pos_data['xyz']
            roll, pitch, yaw = pos_data['rpy']
            
            rospy.loginfo(f"Calculating for {pos_name}: ({x:.3f}, {y:.3f}, {z:.3f})")
            
            # Use mathematical IK
            joint_angles = math_ik.solve(x, y, z, roll, pitch, yaw)
            
            if joint_angles is not None:
                joint_angles_list.append(joint_angles)
                rospy.loginfo(f"✓ Math IK solution found for {pos_name}")
                
                # Log joint angles
                joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
                for name, angle in zip(joint_names, joint_angles):
                    rospy.loginfo(f"  {name}: {degrees(angle):.2f}°")
            else:
                # Fallback to original hardcoded angles
                rospy.logwarn(f"✗ Math IK failed for {pos_name}, using fallback")
                fallback_angles = get_fallback_joint_angles(pos_name)
                joint_angles_list.append(fallback_angles)
        else:
            rospy.logerr(f"Position {pos_name} not found in configuration")
    
    return joint_angles_list

def get_fallback_joint_angles(position_name):
    '''Fallback to original joint angles if math IK fails'''
    fallback_angles = {
        'home': [radians(0), radians(-90), radians(-90), radians(-90), radians(90), radians(0)],
        'red_bin': [radians(65), radians(-55), radians(80), radians(-115), radians(-90), radians(0)],
        'green_bin': [radians(0), radians(-55), radians(80), radians(-115), radians(-90), radians(0)],
        'blue_bin': [radians(-95), radians(-55), radians(80), radians(-115), radians(-90), radians(0)]
    }
    return fallback_angles.get(position_name, fallback_angles['home'])

def move_to_cartesian_with_math_ik(x, y, z, roll=0, pitch=radians(90), yaw=0):
    '''
    Move robot to cartesian position using mathematical IK.
    
    Args:
        x, y, z (float): Target position in meters
        roll, pitch, yaw (float): Target orientation in radians
    
    Returns:
        bool: True if successful, False otherwise
    '''
    # Calculate joint angles using mathematical IK
    joint_angles = math_ik.solve(x, y, z, roll, pitch, yaw)
    
    if joint_angles:
        rospy.loginfo(f'\033[94m>>> Moving to: ({x:.3f}, {y:.3f}, {z:.3f}) using math IK\033[0m')
        return ur5.set_joint_angles(joint_angles)
    else:
        rospy.logerr('\033[91m>>> No math IK solution found\033[0m')
        return False

def box_plan_with_math_ik(box_name, box_length, vacuum_gripper_width):
    '''
    Pick planning using mathematical IK for precise positioning.
    '''
    delta = vacuum_gripper_width + (box_length/2)
    approach_height = 0.15
    
    try:
        # Get box transform (same as original)
        if('Red' in box_name):
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_r1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_r2_frame",
                                                                       rospy.Time(0))
            color_log = '\033[91m' + "Red Package"
            
        elif('Green' in box_name):
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_g1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_g2_frame",
                                                                       rospy.Time(0))
            color_log = '\033[92m' + "Green Package"
            
        elif('Blue' in box_name):
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_b1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_b2_frame",
                                                                       rospy.Time(0))
            color_log = '\033[94m' + "Blue Package"

        x, y, z = box_trans[0], box_trans[1], box_trans[2]
        
        rospy.loginfo(f"Box detected at: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Check if position is reachable
        approach_z = z + delta + approach_height
        pick_z = z + delta
        
        if not math_ik.check_reachability(x, y, approach_z):
            rospy.logwarn("Approach position not reachable, trying different strategy")
            approach_z = z + delta + 0.05  # Lower approach height
        
        # Step 1: Move to approach position
        success = move_to_cartesian_with_math_ik(x, y, approach_z, -0.5, -0.5, 0.5)
        if not success:
            rospy.logerr("Failed to move to approach position")
            return
        
        # Step 2: Move down to pick position
        success = move_to_cartesian_with_math_ik(x, y, pick_z, -0.5, -0.5, 0.5)
        if not success:
            rospy.logerr("Failed to move to pick position")
            return
        
        # Step 3: Activate gripper
        rospy.loginfo("Activating vacuum gripper...")
        os.system('rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
        rospy.sleep(0.5)
        
        # Step 4: Add box to planning scene (same as original)
        box_pose = pose_set(box_trans, box_rot)
        ur5.add_box(box_name, box_length, box_pose)
        ur5.attach_box(box_name)
        
        # Step 5: Move back up
        move_to_cartesian_with_math_ik(x, y, approach_z, -0.5, -0.5, 0.5)
        
        rospy.loginfo(color_log + " Picked using Math IK!" + '\033[0m')
        
    except Exception as e:
        rospy.logerr(f"Math IK box planning failed: {e}")

def bin_plan_with_math_ik(box_name, bin_name):
    '''
    Place planning using mathematical IK.
    '''
    positions = get_robot_positions()
    
    bin_map = {'R': 'red_bin', 'G': 'green_bin', 'B': 'blue_bin'}
    color_map = {'R': '\033[91m' + "Red", 'G': '\033[92m' + "Green", 'B': '\033[94m' + "Blue"}
    
    if bin_name in bin_map:
        position_key = bin_map[bin_name]
        color_log = color_map[bin_name]
        
        if position_key in positions:
            pos_data = positions[position_key]
            x, y, z = pos_data['xyz']
            roll, pitch, yaw = pos_data['rpy']
            
            rospy.loginfo(f"Moving to {color_log} bin using Math IK")
            success = move_to_cartesian_with_math_ik(x, y, z, roll, pitch, yaw)
            
            if not success:
                rospy.logwarn("Math IK movement failed, trying fallback joint angles")
                # Fallback to calculated joint angles
                joint_angles = joint_angles_data()
                bin_index = {'R': 1, 'G': 2, 'B': 3}[bin_name]
                ur5.set_joint_angles(joint_angles[bin_index])
        else:
            rospy.logwarn(f"Position {position_key} not found")
    
    # Deactivate gripper and remove box (same as original)
    rospy.loginfo("Releasing package...")
    os.system('rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
    rospy.sleep(0.5)
    
    ur5.detach_box(box_name)
    ur5.remove_box(box_name)
    
    rospy.loginfo(color_log + " Package Placed using Math IK!" + '\033[0m')

# Keep original function names for compatibility
def camera_callback(msg_camera):
    '''Camera callback (unchanged)'''
    global MasterString, ReadyFlag
    if(ReadyFlag):
        for i in range(0, len(msg_camera.models)):
            MasterString = MasterString + ' ' + msg_camera.models[i].type
        model_list = MasterString.split(' ')
        for i in model_list:
            if('package' in i):
                MasterString = i
                break
    else:
        MasterString = ''

def pose_set(trans, rot):
    '''Pose set function (unchanged)'''
    override = [trans, [-0.5, -0.5, 0.5, 0.5]]
    if(override != []):
        trans = override[0]
        rot = override[1]

    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]
    pose.pose.orientation.x = rot[0]
    pose.pose.orientation.y = rot[1]
    pose.pose.orientation.z = rot[2]
    pose.pose.orientation.w = rot[3]
    return pose

def box_plan(box_name, box_length, vacuum_gripper_width):
    '''Box planning - now uses math IK'''
    box_plan_with_math_ik(box_name, box_length, vacuum_gripper_width)

def bin_plan(box_name, bin_name, bin_joint_angles):
    '''Bin planning - now uses math IK'''
    bin_plan_with_math_ik(box_name, bin_name)

def subscriber_init():
    '''Subscriber initialization (unchanged)'''
    rospy.Subscriber('/mte_roblab/logical_camera_2',
                     LogicalCameraImage,
                     camera_callback)

def test_math_ik_positions():
    '''Test mathematical IK for all key positions'''
    rospy.loginfo('\033[96m' + "=== Testing Mathematical IK for Key Positions ===" + '\033[0m')
    
    positions = get_robot_positions()
    workspace = math_ik.get_workspace_info()
    
    rospy.loginfo(f"UR5 Workspace Limits:")
    rospy.loginfo(f"  Horizontal: {workspace['min_horizontal_reach']:.3f} - {workspace['max_horizontal_reach']:.3f} m")
    rospy.loginfo(f"  Height: {workspace['min_height']:.3f} - {workspace['max_height']:.3f} m")
    
    for name, pos_data in positions.items():
        x, y, z = pos_data['xyz']
        roll, pitch, yaw = pos_data['rpy']
        
        # Check reachability first
        reachable = math_ik.check_reachability(x, y, z)
        
        if reachable:
            joint_angles = math_ik.solve(x, y, z, roll, pitch, yaw)
            if joint_angles:
                rospy.loginfo(f"✓ {name}: ({x:.3f}, {y:.3f}, {z:.3f}) - Math IK successful")
            else:
                rospy.logwarn(f"⚠ {name}: ({x:.3f}, {y:.3f}, {z:.3f}) - Math IK failed (but reachable)")
        else:
            rospy.logwarn(f"✗ {name}: ({x:.3f}, {y:.3f}, {z:.3f}) - Outside workspace")

def controller():
    '''Main controller using mathematical IK'''
    global MasterString, ReadyFlag

    # Test mathematical IK first
    test_math_ik_positions()
    
    # Get joint angles calculated from coordinates
    joint_angles = joint_angles_data()
    
    # Display coordinate information
    positions = get_robot_positions()
    rospy.loginfo('\033[96m' + "=== Using Mathematical IK with Coordinates ===" + '\033[0m')
    for name, pos in positions.items():
        x, y, z = pos['xyz']
        rospy.loginfo(f"{name}: ({x:.3f}, {y:.3f}, {z:.3f})")

    # Go to home position
    rospy.loginfo('\033[96m' + "Moving to home position using Math IK..." + '\033[0m')
    ur5.set_joint_angles(joint_angles[0])

    # Main sorting loop (same logic as original)
    while(not rospy.is_shutdown()):
        if('package_r' in MasterString):
            ReadyFlag = False
            if('1' in MasterString):
                box_plan('Red Box 1', env_values[0], env_values[1])
                bin_plan('Red Box 1', 'R', None)
            else:
                box_plan('Red Box 2', env_values[0], env_values[1])
                bin_plan('Red Box 2', 'R', None)
            ReadyFlag = True
            
        elif('package_g' in MasterString):
            ReadyFlag = False
            if('1' in MasterString):
                box_plan('Green Box 1', env_values[0], env_values[1])
                bin_plan('Green Box 1', 'G', None)
            else:
                box_plan('Green Box 2', env_values[0], env_values[1])
                bin_plan('Green Box 2', 'G', None)
            ReadyFlag = True
            
        elif('package_b' in MasterString):
            ReadyFlag = False
            if('1' in MasterString):
                box_plan('Blue Box 1', env_values[0], env_values[1])
                bin_plan('Blue Box 1', 'B', None)
            else:
                box_plan('Blue Box 2', env_values[0], env_values[1])
                bin_plan('Blue Box 2', 'B', None)
            ReadyFlag = True

        # Return to home using calculated joint angles
        ur5.set_joint_angles(joint_angles[0])

if __name__ == "__main__":
    '''
    Main execution using mathematical IK without modifying lib.py
    '''
    try:
        # Initialize robot with original lib
        ur5 = UR5MoveIt()
        
        # Get environment data
        env_values = env_data()
        
        # Initialize subscribers
        subscriber_init()
        
        # Display startup info
        rospy.loginfo('\033[96m' + "UR5 Sorter with Standalone Mathematical IK" + '\033[0m')
        rospy.loginfo('\033[96m' + "Original lib.py unchanged - using ur5_math_ik module" + '\033[0m')
        rospy.loginfo('\033[96m' + "Modify coordinates in get_robot_positions() to change behavior" + '\033[0m')
        
        # Show workspace info
        workspace = math_ik.get_workspace_info()
        rospy.loginfo(f"Robot workspace: {workspace['min_horizontal_reach']:.3f}m - {workspace['max_horizontal_reach']:.3f}m horizontal")
        rospy.loginfo(f"Height range: {workspace['min_height']:.3f}m - {workspace['max_height']:.3f}m")
        
        # Start main controller
        controller()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
    except Exception as e:
        rospy.logerr(f"Program failed: {e}")
        rospy.loginfo("Check that ur5_math_ik.py is in the same directory")