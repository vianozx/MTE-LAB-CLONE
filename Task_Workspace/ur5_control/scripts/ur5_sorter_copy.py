#! /usr/bin/env python
'''
ROS Script to sort Red, Green and Blue objects on a table into bins.
Enhanced with memory usage and execution time monitoring.
'''

import rospy
import geometry_msgs.msg
import os
import time
import psutil
import gc
import moveit_commander
from env_sim.msg import LogicalCameraImage
from math import radians
from rospy.exceptions import ROSInterruptException
from lib import UR5MoveIt

MasterString = ''  # String to store model names from camera_callback()
ReadyFlag = True  # Flag to indicate ready state of robot

# Performance monitoring variables
start_time = None
operation_times = []
memory_usage = []
planner_usage = {}  # Track which planners are being used
user_selected_planner = None  # Store the user's planner choice for the entire session


def print_performance_stats():
    """
    Print current memory usage and performance statistics.
    """
    process = psutil.Process(os.getpid())
    memory_info = process.memory_info()
    memory_mb = memory_info.rss / 1024 / 1024  # Convert to MB
    cpu_percent = process.cpu_percent()
    
    rospy.loginfo('\033[96m' + "=" * 60 + '\033[0m')
    rospy.loginfo('\033[96m' + f"Memory Usage: {memory_mb:.2f} MB" + '\033[0m')
    rospy.loginfo('\033[96m' + f"Virtual Memory: {memory_info.vms / 1024 / 1024:.2f} MB" + '\033[0m')
    rospy.loginfo('\033[96m' + f"CPU Usage: {cpu_percent:.1f}%" + '\033[0m')
    
    if operation_times:
        avg_time = sum(operation_times) / len(operation_times)
        rospy.loginfo('\033[96m' + f"Average Operation Time: {avg_time:.3f} seconds" + '\033[0m')
        rospy.loginfo('\033[96m' + f"Total Operations: {len(operation_times)}" + '\033[0m')
    
    if memory_usage:
        max_memory = max(memory_usage)
        min_memory = min(memory_usage)
        rospy.loginfo('\033[96m' + f"Peak Memory: {max_memory:.2f} MB" + '\033[0m')
        rospy.loginfo('\033[96m' + f"Min Memory: {min_memory:.2f} MB" + '\033[0m')
    
    # Print planner usage statistics
    if planner_usage:
        rospy.loginfo('\033[96m' + "Motion Planning Algorithm Usage:" + '\033[0m')
        total_plans = sum(planner_usage.values())
        for planner, count in sorted(planner_usage.items(), key=lambda x: x[1], reverse=True):
            percentage = (count / total_plans) * 100
            rospy.loginfo('\033[96m' + f"  {planner}: {count} times ({percentage:.1f}%)" + '\033[0m')
    
    rospy.loginfo('\033[96m' + "=" * 60 + '\033[0m')
    
    # Store current memory usage
    memory_usage.append(memory_mb)


def start_operation_timer():
    """
    Start timing an operation.
    """
    global start_time
    start_time = time.time()
    rospy.loginfo('\033[93m' + f"Starting operation at: {time.strftime('%H:%M:%S')}" + '\033[0m')


def end_operation_timer(operation_name):
    """
    End timing an operation and log the duration.
    
    Parameters:
        operation_name (str): Name of the operation that was completed.
    """
    global start_time
    if start_time is not None:
        end_time = time.time()
        duration = end_time - start_time
        operation_times.append(duration)
        rospy.loginfo('\033[93m' + f"{operation_name} completed in: {duration:.3f} seconds" + '\033[0m')
        start_time = None
        
        # Print performance stats every 5 operations
        if len(operation_times) % 5 == 0:
            print_performance_stats()


def get_current_planner():
    """
    Get the currently selected motion planner from MoveIt.
    
    Returns:
        str: Name of the current planner being used.
    """
    try:
        # Try different methods to get the planner ID
        if hasattr(ur5, '_group') and ur5._group is not None:
            current_planner = ur5._group.get_planner_id()
        elif hasattr(ur5, 'group') and ur5.group is not None:
            current_planner = ur5.group.get_planner_id()
        else:
            current_planner = None
            
        # If planner ID is empty or None, try to get from parameters
        if not current_planner or current_planner == "":
            # Check what's actually configured as default
            try:
                group_config = rospy.get_param('/move_group/ur5_1_planning_group/default_planner_config', None)
                if group_config and group_config != "None":
                    current_planner = group_config
                else:
                    # Get the first available planner from the configuration
                    available_planners = rospy.get_param('/move_group/ur5_1_planning_group/planner_configs', [])
                    if available_planners and len(available_planners) > 0:
                        current_planner = available_planners[0]
                    else:
                        current_planner = "Unknown"
            except:
                current_planner = "Unknown"
                
        return current_planner
    except Exception as e:
        rospy.logwarn(f"Could not retrieve planner info: {e}")
        return "Unknown"


def log_planner_usage_after_motion(operation_type):
    """
    Log which planner was used AFTER the motion is executed.
    This gets the actual planner that MoveIt used for the motion.
    
    Parameters:
        operation_type (str): Type of operation (e.g., "Pick", "Place", "Home")
    """
    current_planner = get_current_planner()
    
    # Update planner usage statistics
    if current_planner in planner_usage:
        planner_usage[current_planner] += 1
    else:
        planner_usage[current_planner] = 1
    
    rospy.loginfo('\033[97m' + f"[PLANNER] {operation_type} motion used: {current_planner}" + '\033[0m')
    return current_planner


def let_moveit_choose_planner(operation_type):
    """
    Let MoveIt choose the planner automatically and log what it will use.
    This doesn't force any specific planner.
    
    Parameters:
        operation_type (str): Type of operation
    
    Returns:
        str: Name of the planner that MoveIt will use
    """
    # Don't set any specific planner - let MoveIt choose
    # Just log what's currently configured
    current_planner = get_current_planner()
    rospy.loginfo('\033[97m' + f"[PLANNER] {operation_type} - MoveIt will choose planner (current: {current_planner})" + '\033[0m')
    return current_planner


def get_planner_from_moveit():
    """
    Alternative method to get planner information from MoveIt parameters.
    
    Returns:
        str: Current planner name from MoveIt configuration.
    """
    try:
        # Try to get the default planner from the planning group configuration
        group_config = rospy.get_param('/move_group/ur5_1_planning_group/default_planner_config', None)
        if group_config and group_config != "None":
            return group_config
        
        # Try to get available planners and use the first one
        available_planners = rospy.get_param('/move_group/ur5_1_planning_group/planner_configs', [])
        if available_planners and len(available_planners) > 0:
            return available_planners[0]
        
        # Fallback to checking the OMPL configuration
        ompl_config = rospy.get_param('/move_group/planner_configs', {})
        if ompl_config:
            # Return the first available planner from configuration
            planner_names = list(ompl_config.keys())
            if planner_names:
                return planner_names[0]
                
        return "RRTConnect"  # Final fallback
        
    except Exception as e:
        rospy.logwarn(f"Could not get planner from MoveIt params: {e}")
        return "RRTConnect"


def get_available_planners():
    """
    Get list of available planners from MoveIt configuration.
    
    Returns:
        list: List of available planner names
    """
    try:
        available_planners = rospy.get_param('/move_group/ur5_1_planning_group/planner_configs', [])
        if not available_planners:
            # Fallback to common planners from your config
            available_planners = ["RRTConnect", "RRTstar", "RRT", "PRM", "PRMstar", "EST", 
                                "KPIECE", "BKPIECE", "LBKPIECE", "FMT", "BFMT", "SPARS", "LazyPRM"]
        return available_planners
    except:
        return ["RRTConnect", "RRTstar", "PRM", "EST", "KPIECE"]  # Fallback list


def ask_user_for_session_planner():
    """
    Ask the user to select a motion planning algorithm for the entire session.
    
    Returns:
        str: Selected planner name for the entire session
    """
    global user_selected_planner
    
    available_planners = get_available_planners()
    
    print(f"\n{'='*70}")
    print(f"🤖 SELECT MOTION PLANNER FOR ENTIRE SESSION")
    print(f"{'='*70}")
    print("This planner will be used for ALL operations in this session.")
    print("\nAvailable Motion Planning Algorithms:")
    
    for i, planner in enumerate(available_planners, 1):
        print(f"  {i}. {planner}")
    
    print(f"  {len(available_planners)+1}. AUTO (Let MoveIt choose automatically)")
    print(f"  {len(available_planners)+2}. RANDOM (Use different random planner for each operation)")
    
    while True:
        try:
            choice = input(f"\nEnter your choice (1-{len(available_planners)+2}): ").strip()
            
            if choice == "":
                print("❌ Please enter a number")
                continue
                
            choice_num = int(choice)
            
            if 1 <= choice_num <= len(available_planners):
                selected_planner = available_planners[choice_num - 1]
                user_selected_planner = selected_planner
                print(f"✅ Selected: {selected_planner} (will be used for ALL operations)")
                return selected_planner
                
            elif choice_num == len(available_planners) + 1:
                user_selected_planner = "AUTO"
                print("✅ Selected: AUTO (MoveIt will choose for each operation)")
                return "AUTO"
                
            elif choice_num == len(available_planners) + 2:
                user_selected_planner = "RANDOM"
                print("✅ Selected: RANDOM (Different random planner for each operation)")
                return "RANDOM"
                
            else:
                print(f"❌ Please enter a number between 1 and {len(available_planners)+2}")
                
        except ValueError:
            print("❌ Please enter a valid number")
        except KeyboardInterrupt:
            print("\n🛑 Operation cancelled by user, defaulting to AUTO")
            user_selected_planner = "AUTO"
            return "AUTO"


def use_session_planner(operation_type):
    """
    Use the planner selected at the beginning of the session for this operation.
    
    Parameters:
        operation_type (str): Type of operation
    
    Returns:
        str: Name of the planner that was set
    """
    global user_selected_planner
    
    if user_selected_planner == "AUTO":
        # Let MoveIt choose automatically
        current_planner = get_current_planner()
        rospy.loginfo('\033[97m' + f"[PLANNER] {operation_type} - Using AUTO mode (current: {current_planner})" + '\033[0m')
        return current_planner
        
    elif user_selected_planner == "RANDOM":
        # Use a random planner for this operation
        import random
        available_planners = get_available_planners()
        selected_planner = random.choice(available_planners)
        
        try:
            if hasattr(ur5, '_group') and ur5._group is not None:
                ur5._group.set_planner_id(selected_planner)
                actual_planner = ur5._group.get_planner_id()
            elif hasattr(ur5, 'group') and ur5.group is not None:
                ur5.group.set_planner_id(selected_planner)
                actual_planner = ur5.group.get_planner_id()
            else:
                actual_planner = selected_planner
                
            rospy.loginfo('\033[97m' + f"[PLANNER] {operation_type} - Using RANDOM: {actual_planner}" + '\033[0m')
            return actual_planner
            
        except Exception as e:
            rospy.logwarn(f"Could not set random planner {selected_planner}: {e}")
            current_planner = get_current_planner()
            rospy.loginfo('\033[97m' + f"[PLANNER] {operation_type} - Fallback to: {current_planner}" + '\033[0m')
            return current_planner
    
    else:
        # Use the specific planner selected by the user
        try:
            if hasattr(ur5, '_group') and ur5._group is not None:
                ur5._group.set_planner_id(user_selected_planner)
                actual_planner = ur5._group.get_planner_id()
            elif hasattr(ur5, 'group') and ur5.group is not None:
                ur5.group.set_planner_id(user_selected_planner)
                actual_planner = ur5.group.get_planner_id()
            else:
                actual_planner = user_selected_planner
                
            if not actual_planner or actual_planner != user_selected_planner:
                rospy.logwarn(f"Failed to set {user_selected_planner}, using {actual_planner}")
                actual_planner = user_selected_planner
                
            rospy.loginfo('\033[97m' + f"[PLANNER] {operation_type} - Using session planner: {actual_planner}" + '\033[0m')
            return actual_planner
            
        except Exception as e:
            rospy.logwarn(f"Could not set session planner {user_selected_planner}: {e}")
            current_planner = get_current_planner()
            rospy.loginfo('\033[97m' + f"[PLANNER] {operation_type} - Fallback to: {current_planner}" + '\033[0m')
            return current_planner
    """
    Set a specific planner and log its usage.
    
    Parameters:
        operation_type (str): Type of operation
        preferred_planner (str): Preferred planner to use
    
    Returns:
        str: Name of the planner that was set
    """
    try:
        # Try to set the planner
        if hasattr(ur5, '_group') and ur5._group is not None:
            ur5._group.set_planner_id(preferred_planner)
            actual_planner = ur5._group.get_planner_id()
        elif hasattr(ur5, 'group') and ur5.group is not None:
            ur5.group.set_planner_id(preferred_planner)
            actual_planner = ur5.group.get_planner_id()
        else:
            actual_planner = preferred_planner
            
        if not actual_planner:
            actual_planner = preferred_planner
            
    except Exception as e:
        rospy.logwarn(f"Could not set planner {preferred_planner}: {e}")
        actual_planner = get_planner_from_moveit()
    
    # Update planner usage statistics
    if actual_planner in planner_usage:
        planner_usage[actual_planner] += 1
    else:
        planner_usage[actual_planner] = 1
    
    rospy.loginfo('\033[97m' + f"[PLANNER] {operation_type} motion using: {actual_planner}" + '\033[0m')
    return actual_planner


def env_data():
    '''
    Data of all environment-specific parameters:
    1. Vacuum Gripper Width
    2. Box Size

    Returns:
        (list) All environment-specific data.
    '''
    box_length = 0.15  # Length of the box
    vacuum_gripper_width = 0.117  # Vacuum Gripper Width

    # Return data when called
    return [box_length,
            vacuum_gripper_width]


def joint_angles_data():
    '''
    Data of all joint angles required for various known positions:
    1. Home: Ready-position for picking objects off the conveyor
    2. Red Bin: Red Bin to place Red packages
    3. Green Bin: Green Bin to place Green packages
    4. Blue Bin: Blue Bin to place Blue packages

    Returns: 
        (list) Joint angles for all positions when called.
    '''
    # Home/Ready angles for picking
    home_joint_angles = [radians(0),
                         radians(-90),
                         radians(-90),
                         radians(-90),
                         radians(90),
                         radians(0)]

    # Red Bin angles
    red_bin_joint_angles = [radians(65),
                            radians(-55),
                            radians(80),
                            radians(-115),
                            radians(-90),
                            radians(0)]

    # Green bin angles
    green_bin_joint_angles = [radians(0),
                              radians(-55),
                              radians(80),
                              radians(-115),
                              radians(-90),
                              radians(0)]

    # Blue bin angles
    blue_bin_joint_angles = [radians(-95),
                             radians(-55),
                             radians(80),
                             radians(-115),
                             radians(-90),
                             radians(0)]

    # Return data when called
    return [home_joint_angles,
            red_bin_joint_angles,
            green_bin_joint_angles,
            blue_bin_joint_angles]


def camera_callback(msg_camera):
    '''
    Callback function for Conveyor Logical Camera Subscriber.

    Parameters:
        msg_camera (LogicalCameraImage object): Data about all the objects detected by the Logical Camera.
    '''
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
    '''
    Assigns pose values w.r.t the world-frame to a PoseStamped object.

    Parameters:
        trans (float[]): Translation Values.
        rot (float[]): RPY Rotation Values.

    Returns:
        pose (geometry_msgs.msg.PoseStamped() object): The complete pose with values.
    '''
    # If you want to override any specific values received, use this
    override = [trans, [-0.5, -0.5, 0.5, 0.5]]
    if(override != []):
        trans = override[0]
        rot = override[1]
        print(override)

    # Generating the object
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
    '''
    Pick-planning for the boxes.

    Parameters:
        box_name (str): The colour of the box detected.
        box_length (float): The size of the box.
        vacuum_gripper_width (float): The width of the vacuum gripper.
    '''
    pick_start_time = time.time()
    
    # Offset for end effector placement
    delta = vacuum_gripper_width + (box_length/2)
    try:
        if('Red' in box_name):  # Red box detected
            # Obtaining the TF transform of the box
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_r1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_r2_frame",
                                                                       rospy.Time(0))
            # Execute pick operation
            box_pose = pose_set(box_trans, box_rot)  # Collating pose values
            box_pose.pose.position.z = box_pose.pose.position.z + delta  # Adding Z Offset
            
            # Use the session planner selected at startup
            selected_planner = use_session_planner("Red Box Pick")
            success = ur5.go_to_pose(box_pose)
            
            # Log what planner was actually used after the motion
            actual_planner = log_planner_usage_after_motion("Red Box Pick")
            
            if not success:
                rospy.logwarn("Failed to reach pick position, but continuing with gripper activation")
            # Activate Vacuum Gripper
            os.system(
                'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            # Add the Red box to the planning scene
            box_pose.pose.position.z = box_pose.pose.position.z - delta  # Removing Z Offset
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            # Log the operation
            pick_time = time.time() - pick_start_time
            rospy.loginfo(
                '\033[91m' + f"Red Package Picked in {pick_time:.3f}s using {actual_planner}!" + '\033[0m')
        elif('Green' in box_name):  # Green box detected
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_g1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_g2_frame",
                                                                       rospy.Time(0))

            box_pose = pose_set(box_trans, box_rot)
            box_pose.pose.position.z = box_pose.pose.position.z + delta
            
            # Use the session planner selected at startup
            selected_planner = use_session_planner("Green Box Pick")
            success = ur5.go_to_pose(box_pose)
            
            # Log what planner was actually used after the motion
            actual_planner = log_planner_usage_after_motion("Green Box Pick")
            
            if not success:
                rospy.logwarn("Failed to reach pick position, but continuing with gripper activation")
            os.system(
                'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            box_pose.pose.position.z = box_pose.pose.position.z - delta
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            pick_time = time.time() - pick_start_time
            rospy.loginfo(
                '\033[92m' + f"Green Package Picked in {pick_time:.3f}s using {actual_planner}!" + '\033[0m')
        elif('Blue' in box_name):  # Blue box detected
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_b1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_b2_frame",
                                                                       rospy.Time(0))

            box_pose = pose_set(box_trans, box_rot)
            box_pose.pose.position.z = box_pose.pose.position.z + delta
            
            # Use the session planner selected at startup
            selected_planner = use_session_planner("Blue Box Pick")
            success = ur5.go_to_pose(box_pose)
            
            # Log what planner was actually used after the motion
            actual_planner = log_planner_usage_after_motion("Blue Box Pick")
            
            if not success:
                rospy.logwarn("Failed to reach pick position, but continuing with gripper activation")
            os.system(
                'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            box_pose.pose.position.z = box_pose.pose.position.z - delta
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            pick_time = time.time() - pick_start_time
            rospy.loginfo(
                '\033[94m' + f"Blue Package Picked in {pick_time:.3f}s using {actual_planner}!" + '\033[0m')
    except Exception as e:
        rospy.logerr(f"Error in box_plan: {e}")
        pass


def bin_plan(box_name, bin_name, bin_joint_angles):
    '''
    Place-planning for the bins.

    Parameters:
        bin_name (str): The colour of the bin.
        bin_joint_angles (float[]): The joint angles of the required bin.
    '''
    place_start_time = time.time()
    
    if(bin_name == 'R'):  # Red bin
        # Set joint angles for the bin
        selected_planner = use_session_planner("Red Bin Place")
        success = ur5.set_joint_angles(bin_joint_angles)
        
        # Log what planner was actually used after the motion
        actual_planner = log_planner_usage_after_motion("Red Bin Place")
        
        if not success:
            rospy.logwarn("Failed to reach bin position, but continuing with gripper deactivation")
        # Deactivate the Gripper
        os.system(
            'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        # Remove the box from the planning scene
        ur5.detach_box(box_name)
        ur5.remove_box(box_name)
        # Log the operation
        place_time = time.time() - place_start_time
        rospy.loginfo(
            '\033[91m' + f"Red Package Placed in {place_time:.3f}s using {actual_planner}!" + '\033[0m')
    elif(bin_name == 'G'):  # Green bin
        selected_planner = use_session_planner("Green Bin Place")
        success = ur5.set_joint_angles(bin_joint_angles)
        
        # Log what planner was actually used after the motion
        actual_planner = log_planner_usage_after_motion("Green Bin Place")
        
        if not success:
            rospy.logwarn("Failed to reach bin position, but continuing with gripper deactivation")
        os.system(
            'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        ur5.detach_box(box_name)
        ur5.remove_box(box_name)
        place_time = time.time() - place_start_time
        rospy.loginfo(
            '\033[92m' + f"Green Package Placed in {place_time:.3f}s using {actual_planner}!" + '\033[0m')
    elif(bin_name == 'B'):  # Blue bin
        selected_planner = use_session_planner("Blue Bin Place")
        success = ur5.set_joint_angles(bin_joint_angles)
        
        # Log what planner was actually used after the motion
        actual_planner = log_planner_usage_after_motion("Blue Bin Place")
        
        if not success:
            rospy.logwarn("Failed to reach bin position, but continuing with gripper deactivation")
        os.system(
            'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        ur5.detach_box(box_name)
        ur5.remove_box(box_name)
        place_time = time.time() - place_start_time
        rospy.loginfo(
            '\033[94m' + f"Blue Package Placed in {place_time:.3f}s using {actual_planner}!" + '\033[0m')


def subscriber_init():
    '''
    Definitions and setups of all Subscribers.
    '''
    # Subscriber for Table Logical Camera
    rospy.Subscriber('/mte_roblab/logical_camera_2',
                     LogicalCameraImage,
                     camera_callback)


def controller():
    '''
    Executes the main operations.
    '''
    global MasterString, ReadyFlag

    # Ask user once for the entire session planner choice
    print("\n🚀 Welcome to UR5 Sorter with Performance Monitoring!")
    session_planner = ask_user_for_session_planner()
    
    # Print initial performance stats
    rospy.loginfo('\033[95m' + f"Starting UR5 Sorter using: {session_planner}" + '\033[0m')
    
    # Print available planners
    available_planners = get_available_planners()
    rospy.loginfo('\033[96m' + f"Available planners: {available_planners}" + '\033[0m')
    
    print_performance_stats()

    # Go to a home position in preparation for picking up the packages
    rospy.loginfo('\033[93m' + "Moving to home position..." + '\033[0m')
    home_start_time = time.time()
    
    selected_planner = use_session_planner("Home Position")
    success = ur5.set_joint_angles(joint_angles[0])
    
    # Log what planner was actually used after the motion
    actual_planner = log_planner_usage_after_motion("Home Position")
    
    home_time = time.time() - home_start_time
    if success:
        rospy.loginfo('\033[93m' + f"Home position reached in {home_time:.3f}s using {actual_planner}" + '\033[0m')
    else:
        rospy.logwarn('\033[93m' + f"Failed to reach home position in {home_time:.3f}s using {actual_planner}" + '\033[0m')

    operation_count = 0

    # Execute box-bin planning
    while(not rospy.is_shutdown()):
        if('package_r' in MasterString):  # Detecting Red package
            operation_count += 1
            start_operation_timer()
            ReadyFlag = False  # Operating
            
            rospy.loginfo('\033[91m' + f"Operation #{operation_count}: Processing Red Package" + '\033[0m')
            
            if('1' in MasterString):  # Detecting the specific model
                # Pick operation
                box_plan('Red Box 1', env_values[0], env_values[1])
                bin_plan('Red Box 1', 'R', joint_angles[1])  # Place operation
            else:
                # Pick operation
                box_plan('Red Box 2', env_values[0], env_values[1])
                bin_plan('Red Box 2', 'R', joint_angles[1])  # Place operation
            
            ReadyFlag = True  # Ready for next operation
            end_operation_timer(f"Red Package Operation #{operation_count}")
            
        elif('package_g' in MasterString):
            operation_count += 1
            start_operation_timer()
            ReadyFlag = False
            
            rospy.loginfo('\033[92m' + f"Operation #{operation_count}: Processing Green Package" + '\033[0m')
            
            if('1' in MasterString):
                box_plan('Green Box 1', env_values[0], env_values[1])
                bin_plan('Green Box 1', 'G', joint_angles[2])
            else:
                box_plan('Green Box 2', env_values[0], env_values[1])
                bin_plan('Green Box 2', 'G', joint_angles[2])
            
            ReadyFlag = True
            end_operation_timer(f"Green Package Operation #{operation_count}")
            
        elif('package_b' in MasterString):
            operation_count += 1
            start_operation_timer()
            ReadyFlag = False
            
            rospy.loginfo('\033[94m' + f"Operation #{operation_count}: Processing Blue Package" + '\033[0m')
            
            if('1' in MasterString):
                box_plan('Blue Box 1', env_values[0], env_values[1])
                bin_plan('Blue Box 1', 'B', joint_angles[3])
            else:
                box_plan('Blue Box 2', env_values[0], env_values[1])
                bin_plan('Blue Box 2', 'B', joint_angles[3])
            
            ReadyFlag = True
            end_operation_timer(f"Blue Package Operation #{operation_count}")

        # Return to home position
        home_start_time = time.time()
        
        selected_planner = use_session_planner("Return Home")
        success = ur5.set_joint_angles(joint_angles[0])
        
        # Log what planner was actually used after the motion
        actual_planner = log_planner_usage_after_motion("Return Home")
        
        home_time = time.time() - home_start_time
        if success:
            rospy.loginfo('\033[93m' + f"Returned to home in {home_time:.3f}s using {actual_planner}" + '\033[0m')
        else:
            rospy.logwarn('\033[93m' + f"Failed to return home in {home_time:.3f}s using {actual_planner}" + '\033[0m')
        
        # Perform garbage collection periodically to manage memory
        if operation_count % 10 == 0:
            gc.collect()
            rospy.loginfo('\033[95m' + "Performed garbage collection" + '\033[0m')


if __name__ == "__main__":
    '''
    Controls overall execution
    '''
    try:
        ur5 = UR5MoveIt()  # Initialise class object node

        # Obtain prerequisite data
        joint_angles = joint_angles_data()
        env_values = env_data()

        # Initialise subscribers
        subscriber_init()

        # Start execution
        controller()
        
    except rospy.ROSInterruptException:
        rospy.loginfo('\033[95m' + "Script interrupted by user" + '\033[0m')
        
    finally:
        # Print final performance summary
        rospy.loginfo('\033[95m' + "Final Performance Summary:" + '\033[0m')
        print_performance_stats()
        
        if operation_times:
            total_time = sum(operation_times)
            rospy.loginfo('\033[95m' + f"Total execution time: {total_time:.3f} seconds" + '\033[0m')
            rospy.loginfo('\033[95m' + f"Fastest operation: {min(operation_times):.3f} seconds" + '\033[0m')
            rospy.loginfo('\033[95m' + f"Slowest operation: {max(operation_times):.3f} seconds" + '\033[0m')
            
        # Print final planner usage summary
        if planner_usage:
            rospy.loginfo('\033[95m' + "Final Motion Planner Usage Summary:" + '\033[0m')
            total_plans = sum(planner_usage.values())
            for planner, count in sorted(planner_usage.items(), key=lambda x: x[1], reverse=True):
                percentage = (count / total_plans) * 100
                rospy.loginfo('\033[95m' + f"  {planner}: {count} times ({percentage:.1f}%)" + '\033[0m')