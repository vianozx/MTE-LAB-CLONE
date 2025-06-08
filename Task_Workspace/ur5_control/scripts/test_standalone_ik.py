#!/usr/bin/env python
'''
Test script for standalone mathematical IK module.
Run this to verify everything works before using in ur5_sorter.py
'''

import math

def test_standalone_ik():
    '''Test the standalone IK module'''
    print("=== Testing Standalone Mathematical IK ===")
    
    try:
        # Import the module
        from ur5_math_ik import UR5MathIK, calculate_joint_angles, is_position_reachable
        print("✓ Successfully imported ur5_math_ik module")
        
        # Create solver
        ik_solver = UR5MathIK()
        print("✓ IK solver created")
        
        # Test positions from your application
        test_positions = [
            {'name': 'Red Bin', 'coords': [-0.484695, 0.1129, 1.463709]},
            {'name': 'Green Bin', 'coords': [1.0, 0.0, 0.95]},
            {'name': 'Blue Bin', 'coords': [0.11, -1.0, 0.95]},
        ]
        
        print("\nTesting positions:")
        print("Position\t\tCoordinates\t\tReachable\tIK Solution")
        print("-" * 70)
        
        results = []
        for test in test_positions:
            name = test['name']
            x, y, z = test['coords']
            
            # Check reachability
            reachable = is_position_reachable(x, y, z)
            
            # Calculate IK
            joint_angles = calculate_joint_angles(x, y, z, 0, math.pi/2, 0)
            has_solution = joint_angles is not None
            
            # Format output
            coords_str = f"({x:.2f}, {y:.2f}, {z:.2f})"
            reach_str = "✓ Yes" if reachable else "✗ No"
            ik_str = "✓ Yes" if has_solution else "✗ No"
            
            print(f"{name:<12}\t{coords_str:<15}\t{reach_str:<10}\t{ik_str}")
            
            if has_solution:
                # Show joint angles in degrees for verification
                joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist1', 'Wrist2', 'Wrist3']
                angles_deg = [math.degrees(angle) for angle in joint_angles]
                print(f"  Joint angles: {', '.join([f'{name}:{angle:.1f}°' for name, angle in zip(joint_names, angles_deg)])}")
            
            results.append(has_solution)
            print()
        
        # Summary
        success_count = sum(results)
        total_count = len(results)
        success_rate = success_count / total_count * 100
        
        print(f"=== Test Results ===")
        print(f"Successful calculations: {success_count}/{total_count} ({success_rate:.1f}%)")
        
        if success_rate >= 70:
            print("✅ Mathematical IK is working well!")
            print("You can proceed to use it in ur5_sorter.py")
        else:
            print("⚠ Some positions failed. Check coordinates or workspace limits.")
        
        # Show workspace info
        from ur5_math_ik import get_ur5_workspace
        workspace = get_ur5_workspace()
        print(f"\nUR5 Workspace Limits:")
        print(f"  Horizontal reach: {workspace['min_horizontal_reach']:.3f} - {workspace['max_horizontal_reach']:.3f} m")
        print(f"  Height range: {workspace['min_height']:.3f} - {workspace['max_height']:.3f} m")
        print(f"  Base height: {workspace['base_height']:.3f} m")
        
        return True
        
    except ImportError as e:
        print(f"✗ Failed to import module: {e}")
        print("Make sure ur5_math_ik.py is in the same directory")
        return False
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False

def test_with_original_lib():
    '''Test integration with original lib.py'''
    print("\n=== Testing Integration with Original lib.py ===")
    
    try:
        # Test importing original lib
        from lib import UR5MoveIt
        print("✓ Successfully imported original lib.py")
        
        # Test importing math IK
        from ur5_math_ik import UR5MathIK
        print("✓ Successfully imported ur5_math_ik.py")
        
        # Create instances
        math_ik = UR5MathIK()
        print("✓ Math IK solver created")
        
        # Test a simple calculation
        joint_angles = math_ik.solve(0.3, 0.0, 0.8, 0, math.pi/2, 0)
        
        if joint_angles:
            print("✓ Math IK calculation successful")
            print("Example joint angles (degrees):")
            joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
            for name, angle in zip(joint_names, joint_angles):
                print(f"  {name}: {math.degrees(angle):.2f}°")
            
            print("\nCode snippet for your ur5_sorter.py:")
            print("# Add this to your imports:")
            print("from ur5_math_ik import UR5MathIK")
            print("\n# Create IK solver:")
            print("math_ik = UR5MathIK()")
            print("\n# Use in your code:")
            print("joint_angles = math_ik.solve(x, y, z, roll, pitch, yaw)")
            print("if joint_angles:")
            print("    ur5.set_joint_angles(joint_angles)")
            
            return True
        else:
            print("✗ Math IK calculation failed")
            return False
            
    except Exception as e:
        print(f"✗ Integration test failed: {e}")
        return False

def interactive_calculator():
    '''Interactive IK calculator'''
    print("\n=== Interactive IK Calculator ===")
    print("Enter coordinates to calculate joint angles")
    print("Format: x y z (or 'q' to quit)")
    
    try:
        from ur5_math_ik import UR5MathIK
        ik_solver = UR5MathIK()
        
        while True:
            try:
                user_input = input("\nEnter coordinates (x y z): ").strip()
                if user_input.lower() == 'q':
                    break
                
                coords = list(map(float, user_input.split()))
                if len(coords) != 3:
                    print("Please enter exactly 3 coordinates")
                    continue
                
                x, y, z = coords
                
                # Check reachability first
                if not ik_solver.check_reachability(x, y, z):
                    print(f"⚠ Position ({x}, {y}, {z}) is outside robot workspace")
                
                # Calculate IK
                joint_angles = ik_solver.solve(x, y, z, 0, math.pi/2, 0)
                
                if joint_angles:
                    print(f"✓ IK solution for ({x}, {y}, {z}):")
                    
                    # Show in degrees
                    joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
                    print("Joint angles (degrees):")
                    for name, angle in zip(joint_names, joint_angles):
                        print(f"  {name}: {math.degrees(angle):.2f}°")
                    
                    # Show in radians for code
                    print("\nFor ur5.set_joint_angles():")
                    angles_str = "[" + ", ".join([f"{angle:.4f}" for angle in joint_angles]) + "]"
                    print(f"  {angles_str}")
                else:
                    print(f"✗ No IK solution found for ({x}, {y}, {z})")
                    
            except ValueError:
                print("Invalid input. Please enter numeric values.")
            except KeyboardInterrupt:
                break
        
        print("Calculator finished.")
        
    except ImportError:
        print("Cannot run calculator - ur5_math_ik module not found")

def main():
    '''Main test function'''
    print("UR5 Standalone Mathematical IK Test")
    print("=" * 50)
    
    # Run basic tests
    basic_success = test_standalone_ik()
    
    if basic_success:
        integration_success = test_with_original_lib()
        
        if integration_success:
            print("\n🎉 All tests passed! Your setup is ready.")
            print("\nNext steps:")
            print("1. Copy ur5_math_ik.py to your scripts directory")
            print("2. Use the modified ur5_sorter.py")
            print("3. Adjust coordinates in get_robot_positions() as needed")
            
            # Offer interactive calculator
            response = input("\nWould you like to try the interactive calculator? (y/n): ")
            if response.lower() == 'y':
                interactive_calculator()
        else:
            print("\n❌ Integration test failed. Check your lib.py file.")
    else:
        print("\n❌ Basic tests failed. Check ur5_math_ik.py file.")

if __name__ == "__main__":
    main()