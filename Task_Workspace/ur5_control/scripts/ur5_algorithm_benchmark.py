#! /usr/bin/env python
'''
ROS Script to benchmark different motion planning algorithms
for UR5 pick and place operations.
'''

import rospy
import time
import csv
import os
from lib import UR5MoveIt
from ur5_sorter import env_data, joint_angles_data

class UR5PlanningBenchmark:
    def __init__(self):
        '''Initialize the benchmark class'''
        self.ur5 = UR5MoveIt()
        
        # Get environment and joint data
        self.joint_angles = joint_angles_data()
        self.env_values = env_data()
        
        # Algorithms to benchmark
        self.algorithms = [
            "RRTConnect",  # Default
            "RRT",
            "RRTstar",
            "PRM",
            "PRMstar",
            "KPIECE",
            "BKPIECE",
            "LBKPIECE",
            "SBL"
        ]
        
        # Results storage
        self.results = []
        
    def set_planning_algorithm(self, algorithm):
        '''
        Set the planning algorithm for MoveIt!
        
        Parameters:
            algorithm (str): Name of the algorithm to use
        '''
        # These parameter paths may need adjustment based on your MoveIt! configuration
        rospy.set_param("/move_group/planning_pipeline/planner_configs/default", algorithm)
        rospy.set_param("/move_group/planning_config/planner_configs/default", algorithm)
        
        # Give time for parameters to take effect
        rospy.sleep(1.0)
        
        # Configure the planner through the MoveIt! interface
        self.ur5.moveit_commander.move_group.set_planner_id(algorithm)
        
        rospy.loginfo(f"Planning algorithm set to: {algorithm}")
        
    def run_single_test(self, algorithm, repetitions=5):
        '''
        Run a test with a specific algorithm
        
        Parameters:
            algorithm (str): Name of the algorithm to test
            repetitions (int): Number of times to repeat the test
        
        Returns:
            dict: Results with metrics
        '''
        self.set_planning_algorithm(algorithm)
        
        metrics = {
            "algorithm": algorithm,
            "success_count": 0,
            "total_planning_time": 0,
            "avg_planning_time": 0,
            "max_planning_time": 0,
            "min_planning_time": float('inf'),
            "total_execution_time": 0,
            "avg_execution_time": 0,
            "total_path_length": 0,
            "avg_path_length": 0
        }
        
        for i in range(repetitions):
            rospy.loginfo(f"Running test {i+1}/{repetitions} for {algorithm}")
            
            try:
                # Go to home position
                start_time = time.time()
                plan_success, plan, planning_time, _ = self.ur5.go_to_joint_angles_plan_only(
                    self.joint_angles[0])
                
                if plan_success:
                    # Measure execution time and path length
                    execution_start = time.time()
                    self.ur5.go_to_joint_angles_execute_plan(plan)
                    execution_time = time.time() - execution_start
                    
                    # Calculate path length (simplified - just counts waypoints)
                    path_length = len(plan.joint_trajectory.points)
                    
                    # Update metrics
                    metrics["success_count"] += 1
                    metrics["total_planning_time"] += planning_time
                    metrics["max_planning_time"] = max(metrics["max_planning_time"], planning_time)
                    metrics["min_planning_time"] = min(metrics["min_planning_time"], planning_time)
                    metrics["total_execution_time"] += execution_time
                    metrics["total_path_length"] += path_length
                    
                    rospy.loginfo(f"Test successful: Planning time: {planning_time:.4f}s, " +
                                 f"Execution time: {execution_time:.4f}s, Path points: {path_length}")
                else:
                    rospy.logerr(f"Planning failed for {algorithm} on attempt {i+1}")
            
            except Exception as e:
                rospy.logerr(f"Error in test: {str(e)}")
                
        # Calculate averages
        if metrics["success_count"] > 0:
            metrics["avg_planning_time"] = metrics["total_planning_time"] / metrics["success_count"]
            metrics["avg_execution_time"] = metrics["total_execution_time"] / metrics["success_count"]
            metrics["avg_path_length"] = metrics["total_path_length"] / metrics["success_count"]
        
        if metrics["min_planning_time"] == float('inf'):
            metrics["min_planning_time"] = 0
            
        return metrics
    
    def run_benchmark(self, repetitions=5):
        '''
        Run benchmarks for all algorithms
        
        Parameters:
            repetitions (int): Number of repetitions per algorithm
        '''
        rospy.loginfo("Starting algorithm benchmarking")
        
        for algorithm in self.algorithms:
            rospy.loginfo(f"Testing algorithm: {algorithm}")
            result = self.run_single_test(algorithm, repetitions)
            self.results.append(result)
            
        # Save results to CSV
        self.save_results()
        
    def save_results(self):
        '''Save benchmark results to CSV file'''
        filename = f"ur5_planning_benchmark_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = [
                "algorithm", "success_count", "avg_planning_time", 
                "min_planning_time", "max_planning_time", 
                "avg_execution_time", "avg_path_length"
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for result in self.results:
                writer.writerow({k: result[k] for k in fieldnames})
                
        rospy.loginfo(f"Results saved to {filename}")
        
        # Also print results to console
        self.print_results()
    
    def print_results(self):
        '''Print benchmark results'''
        rospy.loginfo("\n===== BENCHMARK RESULTS =====")
        rospy.loginfo(f"{'Algorithm':<10} | {'Success':<7} | {'Avg Plan(s)':<10} | {'Min Plan(s)':<10} | {'Max Plan(s)':<10} | {'Avg Exec(s)':<10} | {'Path Len':<8}")
        rospy.loginfo("-" * 80)
        
        for result in self.results:
            rospy.loginfo(
                f"{result['algorithm']:<10} | "
                f"{result['success_count']:<7} | "
                f"{result['avg_planning_time']:<10.4f} | "
                f"{result['min_planning_time']:<10.4f} | "
                f"{result['max_planning_time']:<10.4f} | "
                f"{result['avg_execution_time']:<10.4f} | "
                f"{result['avg_path_length']:<8.1f}"
            )

if __name__ == "__main__":
    '''Main function to run benchmarks'''
    try:
        rospy.init_node('ur5_algorithm_benchmark')
        
        # Create benchmark runner
        benchmark = UR5PlanningBenchmark()
        
        # Run benchmarks with 5 repetitions per algorithm
        benchmark.run_benchmark(repetitions=5)
        
        rospy.loginfo("Benchmarking complete!")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupted")