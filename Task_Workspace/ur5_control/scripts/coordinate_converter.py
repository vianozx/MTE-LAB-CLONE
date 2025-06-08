#!/usr/bin/env python
'''
Coordinate converter and workspace analyzer for your UR5 setup.
Helps understand the relationship between world and robot coordinates.
'''

import math

def analyze_your_setup():
    '''Analyze your specific robot setup'''
    print("=== Your UR5 Setup Analysis ===")
    
    # From your URDF
    robot_base_world_z = 0.95  # Robot base is 0.95m above world origin
    
    # UR5 parameters
    d1 = 0.089159    # Base height (above robot base)
    d6 = 0.0823      # Tool length
    a2 = 0.425       # Upper arm
    a3 = 0.39225     # Forearm
    
    print(f"Robot base position in world: (0, 0, {robot_base_world_z})")
    print(f"Robot base height: {d1:.3f} m")
    print(f"Tool length: {d6:.3f} m")
    print(f"Upper arm length: {a2:.3f} m")
    print(f"Forearm length: {a3:.3f} m")
    
    # Calculate key heights in world coordinates
    shoulder_height_world = robot_base_world_z + d1
    max_reach_height_world = shoulder_height_world + a2 + a3 + d6
    min_reach_height_world = shoulder_height_world - a2 - a3 + d6
    
    print(f"\n=== Key Heights in World Coordinates ===")
    print(f"World origin (floor): 0.000 m")
    print(f"Robot base: {robot_base_world_z:.3f} m")
    print(f"Robot shoulder: {shoulder_height_world:.3f} m")
    print(f"Maximum reach height: {max_reach_height_world:.3f} m")
    print(f"Minimum reach height: {min_reach_height_world:.3f} m")
    
    # Analyze your current pose
    your_pose_z = 1.039159
    robot_relative_z = your_pose_z - robot_base_world_z
    
    print(f"\n=== Your Current Pose Analysis ===")
    print(f"Your end effector Z (world): {your_pose_z:.6f} m")
    print(f"Your end effector Z (robot): {robot_relative_z:.6f} m")
    print(f"Above robot base by: {robot_relative_z:.3f} m")
    print(f"Above shoulder by: {robot_relative_z - d1:.3f} m")
    
    if min_reach_height_world <= your_pose_z <= max_reach_height_world:
        print("✅ Your pose Z is within reachable height range")
    else:
        print("❌ Your pose Z is outside reachable height range")
    
    return {
        'robot_base_world_z': robot_base_world_z,
        'shoulder_height_world': shoulder_height_world,
        'max_height_world': max_reach_height_world,
        'min_height_world': min_reach_height_world,
        'your_pose_z': your_pose_z,
        'robot_relative_z': robot_relative_z
    }

def convert_coordinates():
    '''Interactive coordinate converter'''
    print("\n=== Interactive Coordinate Converter ===")
    print("Convert between world coordinates and robot-relative coordinates")
    
    robot_base_world_z = 0.95
    
    print(f"Robot base is at (0, 0, {robot_base_world_z}) in world coordinates")
    print("Enter 'q' to quit")
    
    while True:
        try:
            user_input = input("\nEnter world coordinates (x y z): ").strip()
            if user_input.lower() == 'q':
                break
            
            coords = list(map(float, user_input.split()))
            if len(coords) != 3:
                print("Please enter exactly 3 coordinates")
                continue
            
            world_x, world_y, world_z = coords
            
            # Convert to robot coordinates
            robot_x = world_x - 0.0  # Robot base X offset
            robot_y = world_y - 0.0  # Robot base Y offset  
            robot_z = world_z - robot_base_world_z  # Robot base Z offset
            
            print(f"World coordinates: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
            print(f"Robot coordinates: ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})")
            
            # Check reachability
            horizontal_dist = math.sqrt(robot_x**2 + robot_y**2)
            max_horizontal = 0.425 + 0.39225  # a2 + a3
            
            d1 = 0.089159
            d6 = 0.0823
            max_z = d1 + max_horizontal + d6
            min_z = d1 - max_horizontal + d6
            
            print(f"Horizontal distance: {horizontal_dist:.3f} m (max: {max_horizontal:.3f} m)")
            print(f"Z relative to robot: {robot_z:.3f} m (range: {min_z:.3f} to {max_z:.3f} m)")
            
            if horizontal_dist <= max_horizontal and min_z <= robot_z <= max_z:
                print("✅ Position appears reachable")
            else:
                print("❌ Position may not be reachable")
                
        except ValueError:
            print("Invalid input. Please enter numeric values.")
        except KeyboardInterrupt:
            break

def test_common_positions():
    '''Test common positions in your workspace'''
    print("\n=== Testing Common Positions ===")
    
    # Import the corrected IK solver
    try:
        from ur5_math_ik import UR5MathIK
        solver = UR5MathIK()
        print("✅ Using corrected IK solver")
    except:
        print("❌ Could not import corrected IK solver")
        return
    
    # Test positions that should work in your setup
    test_positions = [
        {'name': 'Your Current Pose', 'coords': [-0.000003, 0.000014, 1.039159]},
        {'name': 'Home Position', 'coords': [0.0, 0.0, 1.2]},
        {'name': 'Table Level', 'coords': [0.3, 0.0, 1.0]},  # Just above robot base
        {'name': 'Higher Position', 'coords': [0.2, 0.0, 1.3]},
        {'name': 'Lower Position', 'coords': [0.4, 0.0, 1.0]},
        {'name': 'Right Side', 'coords': [0.2, 0.3, 1.1]},
        {'name': 'Left Side', 'coords': [0.2, -0.3, 1.1]},
    ]
    
    print("Position\t\tWorld Coords\t\tReachable\tIK Solution")
    print("-" * 70)
    
    for test in test_positions:
        name = test['name']
        world_x, world_y, world_z = test['coords']
        
        # Check reachability
        reachable = solver.check_reachability(world_x, world_y, world_z)
        
        # Try IK
        joint_angles = solver.solve(world_x, world_y, world_z, 0, math.pi/2, 0)
        has_solution = joint_angles is not None
        
        coords_str = f"({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
        reach_str = "✅ Yes" if reachable else "❌ No"
        ik_str = "✅ Yes" if has_solution else "❌ No"
        
        print(f"{name:<15}\t{coords_str:<15}\t{reach_str:<12}\t{ik_str}")
        
        if has_solution and name == 'Your Current Pose':
            print("  ^ This should work since it's your current robot pose!")

def main():
    '''Main function'''
    print("UR5 Coordinate System Analysis")
    print("=" * 50)
    
    # Analyze the setup
    setup_info = analyze_your_setup()
    
    # Test positions
    test_common_positions()
    
    # Offer interactive converter
    response = input("\nWould you like to try the coordinate converter? (y/n): ")
    if response.lower() == 'y':
        convert_coordinates()
    
    print("\n=== Summary ===")
    print("Key points about your setup:")
    print("1. Robot base is 0.95m above world origin")
    print("2. End effector calculations must account for this offset")
    print("3. Your current pose should be reachable with corrected IK")
    print("4. Use the corrected ur5_math_ik.py for proper calculations")

if __name__ == "__main__":
    main()