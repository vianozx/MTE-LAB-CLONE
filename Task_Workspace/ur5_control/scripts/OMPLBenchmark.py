#! /usr/bin/env python
'''
Interactive OMPL Planner Benchmarking Script - Modified from ur5_sorter.py
This script asks you to choose a specific OMPL planner and benchmarks only that planner
with the available boxes in the scene.
'''

import rospy
import geometry_msgs.msg
import os
import time
import math
import psutil
import csv
import numpy as np
from collections import defaultdict
from env_sim.msg import LogicalCameraImage
from math import radians
from rospy.exceptions import ROSInterruptException
from lib import UR5MoveIt

# Global variables for camera and benchmarking
MasterString = ''
ReadyFlag = True
benchmark_results = []
boxes_processed_count = 0

# List of available planners
AVAILABLE_PLANNERS = [
    'RRTConnect', 'RRT', 'RRTstar', 'EST', 'KPIECE', 'SBL', 'LBKPIECE', 
    'BKPIECE', 'TRRT', 'PRM', 'PRMstar', 'FMT', 'BFMT', 'PDST',
    'STRIDE', 'BiTRRT', 'LBTRRT', 'BiEST', 'ProjEST', 'LazyPRM',
    'LazyPRMstar', 'SPARS', 'SPARStwo'
]

def choose_planner():
    """Interactive planner selection"""
    print("\n" + "="*60)
    print("OMPL PLANNER SELECTION")
    print("="*60)
    print("Available OMPL Planners:")
    
    for i, planner in enumerate(AVAILABLE_PLANNERS, 1):
        print(f"  {i:2d}. {planner}")
    
    while True:
        try:
            choice = input(f"\nChoose a planner (1-{len(AVAILABLE_PLANNERS)}) or type planner name: ").strip()
            
            # Check if it's a number
            if choice.isdigit():
                choice_num = int(choice)
                if 1 <= choice_num <= len(AVAILABLE_PLANNERS):
                    selected_planner = AVAILABLE_PLANNERS[choice_num - 1]
                    break
                else:
                    print(f"Please enter a number between 1 and {len(AVAILABLE_PLANNERS)}")
                    continue
            
            # Check if it's a planner name
            if choice in AVAILABLE_PLANNERS:
                selected_planner = choice
                break
            
            # Check for partial matches
            matches = [p for p in AVAILABLE_PLANNERS if choice.lower() in p.lower()]
            if len(matches) == 1:
                selected_planner = matches[0]
                break
            elif len(matches) > 1:
                print(f"Multiple matches found: {matches}")
                print("Please be more specific.")
                continue
            else:
                print(f"Planner '{choice}' not found. Please try again.")
                continue
                
        except (ValueError, KeyboardInterrupt):
            print("\nExiting...")
            return None
    
    print(f"\nSelected planner: {selected_planner}")
    
    # Ask for number of boxes to process
    while True:
        try:
            num_boxes = input("\nHow many boxes would you like to process? (default: all available): ").strip()
            if num_boxes == "":
                num_boxes = None
                break
            else:
                num_boxes = int(num_boxes)
                if num_boxes > 0:
                    break
                else:
                    print("Please enter a positive number.")
        except ValueError:
            print("Please enter a valid number.")
    
    return selected_planner, num_boxes

def env_data():
    '''Environment-specific parameters'''
    box_length = 0.15
    vacuum_gripper_width = 0.117
    return [box_length, vacuum_gripper_width]

def joint_angles_data():
    '''Joint angles for various positions'''
    home_joint_angles = [radians(0), radians(-90), radians(-90), 
                        radians(-90), radians(90), radians(0)]
    red_bin_joint_angles = [radians(65), radians(-55), radians(80),
                           radians(-115), radians(-90), radians(0)]
    green_bin_joint_angles = [radians(0), radians(-55), radians(80),
                             radians(-115), radians(-90), radians(0)]
    blue_bin_joint_angles = [radians(-95), radians(-55), radians(80),
                            radians(-115), radians(-90), radians(0)]
    
    return [home_joint_angles, red_bin_joint_angles, 
            green_bin_joint_angles, blue_bin_joint_angles]

def camera_callback(msg_camera):
    '''Callback function for Logical Camera'''
    global MasterString, ReadyFlag

    if ReadyFlag:
        MasterString = ''  # Reset string
        for i in range(0, len(msg_camera.models)):
            MasterString = MasterString + ' ' + msg_camera.models[i].type
        model_list = MasterString.split(' ')

        for i in model_list:
            if 'package' in i:
                MasterString = i
                break
    else:
        MasterString = ''

def pose_set(trans, rot):
    '''Create pose object'''
    override = [trans, [-0.5, -0.5, 0.5, 0.5]]
    if override != []:
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

def get_memory_usage():
    """Get current memory usage in MB"""
    process = psutil.Process(os.getpid())
    memory_info = process.memory_info()
    return memory_info.rss / 1024 / 1024

def calculate_cartesian_path_length(trajectory):
    """Calculate approximate Cartesian path length"""
    if not trajectory or not trajectory.joint_trajectory.points:
        return 0.0
    
    total_length = 0.0
    prev_positions = None
    
    for point in trajectory.joint_trajectory.points:
        current_positions = point.positions
        
        if prev_positions is not None:
            joint_distance = sum(abs(curr - prev) for curr, prev 
                               in zip(current_positions, prev_positions))
            cartesian_approx = joint_distance * 0.15  # Rough conversion
            total_length += cartesian_approx
        
        prev_positions = current_positions
    
    return total_length

def calculate_joint_space_path_length(trajectory):
    """Calculate joint space path length"""
    if not trajectory or not trajectory.joint_trajectory.points:
        return 0.0
    
    total_length = 0.0
    prev_positions = None
    
    for point in trajectory.joint_trajectory.points:
        current_positions = point.positions
        
        if prev_positions is not None:
            joint_distance = sum(abs(curr - prev) for curr, prev 
                               in zip(current_positions, prev_positions))
            total_length += joint_distance
        
        prev_positions = current_positions
    
    return total_length

def benchmark_motion(ur5, target_joint_angles, operation_name, planner_name, box_name=""):
    """Benchmark a motion using the specified planner"""
    
    # Set the planner
    ur5._group.set_planner_id(planner_name)
    ur5._group.set_planning_time(10.0)
    ur5._group.set_num_planning_attempts(5)
    
    # Measure memory before planning
    memory_before = get_memory_usage()
    
    # Measure planning time
    planning_start_time = time.time()
    
    # Plan the motion
    ur5._group.set_joint_value_target(target_joint_angles)
    plan_result = ur5._group.plan()
    
    planning_end_time = time.time()
    planning_time = planning_end_time - planning_start_time
    
    # Measure memory after planning
    memory_after = get_memory_usage()
    memory_used = memory_after - memory_before
    
    if plan_result[0]:  # Planning successful
        # Calculate path metrics
        cartesian_length = calculate_cartesian_path_length(plan_result[1])
        joint_length = calculate_joint_space_path_length(plan_result[1])
        
        # Measure execution time
        execution_start_time = time.time()
        success = ur5._group.execute(plan_result[1], wait=True)
        ur5._group.stop()
        execution_end_time = time.time()
        
        execution_time = execution_end_time - execution_start_time
        
        # Store benchmark result
        result = {
            'planner': planner_name,
            'operation': operation_name,
            'box': box_name,
            'success': success,
            'planning_time': planning_time,
            'execution_time': execution_time,
            'cartesian_path_length': cartesian_length,
            'joint_path_length': joint_length,
            'memory_usage': memory_used,
            'timestamp': time.time()
        }
        
        benchmark_results.append(result)
        
        print(f"    {operation_name}: Success={success}, "
              f"Plan={planning_time:.3f}s, Exec={execution_time:.3f}s, "
              f"CartPath={cartesian_length:.3f}m, JointPath={joint_length:.3f}rad, "
              f"Memory={memory_used:.2f}MB")
        
        return success
    else:
        # Planning failed
        result = {
            'planner': planner_name,
            'operation': operation_name,
            'box': box_name,
            'success': False,
            'planning_time': planning_time,
            'execution_time': 0,
            'cartesian_path_length': 0,
            'joint_path_length': 0,
            'memory_usage': memory_used,
            'timestamp': time.time()
        }
        
        benchmark_results.append(result)
        
        print(f"    {operation_name}: PLANNING FAILED (Plan time: {planning_time:.3f}s)")
        return False

def box_plan(box_name, box_length, vacuum_gripper_width, ur5, planner_name):
    '''Pick operation with benchmarking'''
    delta = vacuum_gripper_width + (box_length/2)
    
    try:
        if 'Red' in box_name or 'r' in box_name:
            # Get transform
            if '1' in box_name:
                frame_name = "/logical_camera_2_package_r1_frame"
            else:
                frame_name = "/logical_camera_2_package_r2_frame"
                
            (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world", frame_name, rospy.Time(0))
            color = "red"
            
        elif 'Green' in box_name or 'g' in box_name:
            if '1' in box_name:
                frame_name = "/logical_camera_2_package_g1_frame"
            else:
                frame_name = "/logical_camera_2_package_g2_frame"
                
            (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world", frame_name, rospy.Time(0))
            color = "green"
            
        elif 'Blue' in box_name or 'b' in box_name:
            if '1' in box_name:
                frame_name = "/logical_camera_2_package_b1_frame"
            else:
                frame_name = "/logical_camera_2_package_b2_frame"
                
            (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world", frame_name, rospy.Time(0))
            color = "blue"
        else:
            print(f"Unknown box type: {box_name}")
            return False
        
        # Create pose for picking
        box_pose = pose_set(box_trans, box_rot)
        box_pose.pose.position.z = box_pose.pose.position.z + delta
        
        # Plan and execute pick motion
        ur5._group.set_planner_id(planner_name)
        ur5._group.set_pose_target(box_pose)
        
        planning_start = time.time()
        plan_result = ur5._group.plan()
        planning_time = time.time() - planning_start
        
        if plan_result[0]:
            cartesian_length = calculate_cartesian_path_length(plan_result[1])
            joint_length = calculate_joint_space_path_length(plan_result[1])
            
            execution_start = time.time()
            ur5._group.execute(plan_result[1], wait=True)
            ur5._group.stop()
            execution_time = time.time() - execution_start
            
            # Record metrics
            result = {
                'planner': planner_name,
                'operation': f'pick_{color}',
                'box': box_name,
                'success': True,
                'planning_time': planning_time,
                'execution_time': execution_time,
                'cartesian_path_length': cartesian_length,
                'joint_path_length': joint_length,
                'memory_usage': 0,
                'timestamp': time.time()
            }
            benchmark_results.append(result)
            
            print(f"    pick_{color}: Success=True, "
                  f"Plan={planning_time:.3f}s, Exec={execution_time:.3f}s, "
                  f"CartPath={cartesian_length:.3f}m, JointPath={joint_length:.3f}rad")
            
            # Activate gripper and add to scene
            os.system('rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            box_pose.pose.position.z = box_pose.pose.position.z - delta
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            
            rospy.loginfo(f'\033[9{1 if color=="red" else 2 if color=="green" else 4}m' + 
                         f"{color.title()} Package Picked!" + '\033[0m')
            return True
        else:
            # Record failed pick
            result = {
                'planner': planner_name,
                'operation': f'pick_{color}',
                'box': box_name,
                'success': False,
                'planning_time': planning_time,
                'execution_time': 0,
                'cartesian_path_length': 0,
                'joint_path_length': 0,
                'memory_usage': 0,
                'timestamp': time.time()
            }
            benchmark_results.append(result)
            print(f"    pick_{color}: PLANNING FAILED")
            return False
            
    except Exception as e:
        print(f"Pick operation failed: {e}")
        return False

def bin_plan(box_name, bin_name, bin_joint_angles, ur5, planner_name):
    '''Place operation with benchmarking'''
    color_map = {'R': 'red', 'G': 'green', 'B': 'blue'}
    color = color_map.get(bin_name, bin_name.lower())
    
    success = benchmark_motion(ur5, bin_joint_angles, f'place_{color}', planner_name, box_name)
    
    if success:
        # Deactivate gripper and clean up scene
        os.system('rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        ur5.detach_box(box_name)
        ur5.remove_box(box_name)
        
        color_code = 91 if bin_name == 'R' else 92 if bin_name == 'G' else 94
        rospy.loginfo(f'\033[{color_code}m' + f"{color.title()} Package Placed!" + '\033[0m')
        
    return success

def save_benchmark_results(planner_name):
    """Save benchmark results to CSV files"""
    if not benchmark_results:
        print("No benchmark results to save")
        return
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    
    # Save detailed results
    detailed_filename = f"ompl_{planner_name}_benchmark_{timestamp}.csv"
    with open(detailed_filename, 'w', newline='') as csvfile:
        fieldnames = [
            'planner', 'operation', 'box', 'success', 'planning_time',
            'execution_time', 'cartesian_path_length', 'joint_path_length',
            'memory_usage', 'timestamp'
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        for result in benchmark_results:
            writer.writerow(result)
    
    print(f"\nDetailed results saved to: {detailed_filename}")
    
    # Calculate summary statistics
    total_operations = len(benchmark_results)
    successful_operations = sum(1 for r in benchmark_results if r['success'])
    success_rate = (successful_operations / total_operations * 100) if total_operations > 0 else 0
    
    successful_results = [r for r in benchmark_results if r['success']]
    
    if successful_results:
        avg_planning_time = sum(r['planning_time'] for r in successful_results) / len(successful_results)
        avg_execution_time = sum(r['execution_time'] for r in successful_results) / len(successful_results)
        avg_cartesian_length = sum(r['cartesian_path_length'] for r in successful_results) / len(successful_results)
        avg_joint_length = sum(r['joint_path_length'] for r in successful_results) / len(successful_results)
        total_time = sum(r['planning_time'] + r['execution_time'] for r in successful_results)
    else:
        avg_planning_time = avg_execution_time = avg_cartesian_length = avg_joint_length = total_time = 0
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"BENCHMARK SUMMARY FOR {planner_name}")
    print(f"{'='*60}")
    print(f"Total Operations: {total_operations}")
    print(f"Successful Operations: {successful_operations}")
    print(f"Success Rate: {success_rate:.1f}%")
    if successful_results:
        print(f"Average Planning Time: {avg_planning_time:.3f}s")
        print(f"Average Execution Time: {avg_execution_time:.3f}s")
        print(f"Average Cartesian Path Length: {avg_cartesian_length:.3f}m")
        print(f"Average Joint Path Length: {avg_joint_length:.3f}rad")
        print(f"Total Time for Successful Operations: {total_time:.3f}s")
    
    # Count operations by type
    operation_counts = defaultdict(int)
    operation_successes = defaultdict(int)
    
    for result in benchmark_results:
        operation_counts[result['operation']] += 1
        if result['success']:
            operation_successes[result['operation']] += 1
    
    print(f"\nOperation Breakdown:")
    for operation in sorted(operation_counts.keys()):
        success_count = operation_successes[operation]
        total_count = operation_counts[operation]
        success_percent = (success_count / total_count * 100) if total_count > 0 else 0
        print(f"  {operation}: {success_count}/{total_count} ({success_percent:.1f}%)")

def subscriber_init():
    '''Initialize subscribers'''
    rospy.Subscriber('/mte_roblab/logical_camera_2',
                     LogicalCameraImage,
                     camera_callback)

def get_available_boxes():
    """Wait and get list of available boxes"""
    print("\nScanning for available boxes...")
    rospy.sleep(3)  # Wait for camera data
    
    # Get fresh camera data
    global MasterString
    available_boxes = []
    
    # Check multiple times to get all boxes
    for _ in range(5):
        rospy.sleep(0.5)
        if 'package' in MasterString:
            if MasterString not in available_boxes:
                available_boxes.append(MasterString)
    
    return available_boxes

def controller():
    '''Main controller with interactive planner selection'''
    global MasterString, ReadyFlag, boxes_processed_count
    
    # Choose planner interactively
    choice_result = choose_planner()
    if choice_result is None:
        return
    
    selected_planner, max_boxes = choice_result
    
    print(f"\nInitializing UR5 controller...")
    ur5 = UR5MoveIt()
    joint_angles = joint_angles_data()
    env_values = env_data()
    
    print(f"\n{'='*60}")
    print(f"BENCHMARKING PLANNER: {selected_planner}")
    print(f"{'='*60}")
    
    # Go to home position
    print(f"Moving to home position...")
    benchmark_motion(ur5, joint_angles[0], 'initial_home', selected_planner)
    
    # Get available boxes
    available_boxes = get_available_boxes()
    if not available_boxes:
        print("No boxes detected! Make sure boxes are spawned and camera is working.")
        return
    
    print(f"Available boxes detected: {available_boxes}")
    
    if max_boxes:
        print(f"Will process maximum {max_boxes} boxes")
    else:
        print(f"Will process all available boxes")
    
    # Process boxes
    boxes_processed_count = 0
    
    while not rospy.is_shutdown():
        if max_boxes and boxes_processed_count >= max_boxes:
            print(f"Reached maximum number of boxes ({max_boxes})")
            break
            
        current_box = MasterString
        if 'package_r' in current_box:
            ReadyFlag = False
            print(f"\n  Processing Red package: {current_box}")
            
            box_number = "1" if "1" in current_box else "2"
            box_display_name = f"Red Box {box_number}"
            
            if box_plan(current_box, env_values[0], env_values[1], ur5, selected_planner):
                bin_plan(box_display_name, 'R', joint_angles[1], ur5, selected_planner)
            
            boxes_processed_count += 1
            ReadyFlag = True
            
        elif 'package_g' in current_box:
            ReadyFlag = False
            print(f"\n  Processing Green package: {current_box}")
            
            box_number = "1" if "1" in current_box else "2"
            box_display_name = f"Green Box {box_number}"
            
            if box_plan(current_box, env_values[0], env_values[1], ur5, selected_planner):
                bin_plan(box_display_name, 'G', joint_angles[2], ur5, selected_planner)
            
            boxes_processed_count += 1
            ReadyFlag = True
            
        elif 'package_b' in current_box:
            ReadyFlag = False
            print(f"\n  Processing Blue package: {current_box}")
            
            box_number = "1" if "1" in current_box else "2"
            box_display_name = f"Blue Box {box_number}"
            
            if box_plan(current_box, env_values[0], env_values[1], ur5, selected_planner):
                bin_plan(box_display_name, 'B', joint_angles[3], ur5, selected_planner)
            
            boxes_processed_count += 1
            ReadyFlag = True
        
        else:
            if boxes_processed_count == 0:
                print("Waiting for boxes to be detected...")
                rospy.sleep(1)
                continue
            else:
                print("No more boxes detected.")
                break
        
        # Return to home after each operation
        print(f"  Returning to home position...")
        benchmark_motion(ur5, joint_angles[0], 'return_home', selected_planner)
        rospy.sleep(1.0)
    
    # Save results for the selected planner
    save_benchmark_results(selected_planner)
    print(f"\nBenchmark completed for {selected_planner}!")
    print(f"Processed {boxes_processed_count} boxes total.")

if __name__ == "__main__":
    '''Main execution'''
    try:
        rospy.init_node('ompl_interactive_benchmark', anonymous=True)
        
        # Initialize subscribers
        subscriber_init()
        
        # Give time for initialization
        rospy.sleep(2)
        
        # Start the interactive benchmark
        controller()
        
    except rospy.ROSInterruptException:
        print("Benchmark interrupted")
        if benchmark_results:
            save_benchmark_results("interrupted")
    except KeyboardInterrupt:
        print("\nBenchmark interrupted by user")
        if benchmark_results:
            save_benchmark_results("interrupted")