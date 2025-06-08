#!/usr/bin/env python
'''
Batch converter for predefined UR5 positions.
Calculates joint angles for all key positions in your workspace.
'''

import math

def get_predefined_positions():
    '''Define all key positions for your UR5 application'''
    positions = {
       'current_pose': {
            'coords': [0.817242, 0.817242, 0.817242],
            'orientation': [-3.141451, -0.000210, -0.000210],
            'description': 'Your current robot pose'
        },
        'red_bin': {
            'coords': [0.817242, 0.108929, 0.944308],
            'orientation': [0, 90, 0],
            'description': 'Red bin drop position'
        },
        'green_bin': {
            'coords': [1.038921, -0.109227, 1.038994],
            'orientation': [0, 90, -90],
            'description': 'Green bin drop position'
        },
        'blue_bin': {
            'coords': [-0.109161, -0.109161, 1.038921],
            'orientation': [0, 90, 0],
            'description': 'Blue bin drop position'
        }
    }
    return positions

def calculate_all_positions():
    '''Calculate joint angles for all predefined positions'''
    print("UR5 Batch Position Calculator")
    print("=" * 50)
    
    try:
        from ur5_math_ik import UR5MathIK
        ik_solver = UR5MathIK()
        print("✅ IK solver loaded successfully")
    except ImportError:
        print("❌ Error: ur5_math_ik.py not found")
        return None
    
    positions = get_predefined_positions()
    results = {}
    successful = 0
    total = len(positions)
    
    print(f"\nCalculating joint angles for {total} positions...")
    print()
    
    # Process each position
    for name, pos_data in positions.items():
        print(f"--- {name.upper().replace('_', ' ')} ---")
        print(f"Description: {pos_data['description']}")
        
        x, y, z = pos_data['coords']
        roll_deg, pitch_deg, yaw_deg = pos_data['orientation']
        
        print(f"World coordinates: ({x:.3f}, {y:.3f}, {z:.3f})")
        print(f"Orientation: roll={roll_deg}°, pitch={pitch_deg}°, yaw={yaw_deg}°")
        
        # Convert to radians
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        
        # Check reachability
        reachable = ik_solver.check_reachability(x, y, z)
        print(f"Reachable: {'✅ Yes' if reachable else '❌ No'}")
        
        if reachable:
            # Calculate joint angles
            joint_angles = ik_solver.solve(x, y, z, roll, pitch, yaw)
            
            if joint_angles:
                successful += 1
                print("✅ IK solution found")
                
                # Store results
                results[name] = {
                    'world_coords': pos_data['coords'],
                    'orientation_deg': pos_data['orientation'],
                    'joint_angles_rad': joint_angles,
                    'joint_angles_deg': [math.degrees(angle) for angle in joint_angles],
                    'description': pos_data['description'],
                    'success': True
                }
                
                # Show joint angles in degrees
                joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist1', 'Wrist2', 'Wrist3']
                print("Joint angles (degrees):")
                for joint_name, angle in zip(joint_names, joint_angles):
                    print(f"  {joint_name}: {math.degrees(angle):>7.2f}°")
            else:
                print("❌ No IK solution found")
                results[name] = {
                    'world_coords': pos_data['coords'],
                    'orientation_deg': pos_data['orientation'],
                    'description': pos_data['description'],
                    'success': False,
                    'error': 'No IK solution'
                }
        else:
            print("❌ Position outside workspace")
            results[name] = {
                'world_coords': pos_data['coords'],
                'orientation_deg': pos_data['orientation'],
                'description': pos_data['description'],
                'success': False,
                'error': 'Outside workspace'
            }
        
        print()
    
    # Summary
    print("=" * 50)
    print(f"SUMMARY: {successful}/{total} positions successful ({successful/total*100:.1f}%)")
    print()
    
    # List successful positions
    print("✅ SUCCESSFUL POSITIONS:")
    for name, data in results.items():
        if data['success']:
            coords = data['world_coords']
            print(f"  {name}: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})")
    
    # List failed positions
    failed = [name for name, data in results.items() if not data['success']]
    if failed:
        print(f"\n❌ FAILED POSITIONS:")
        for name in failed:
            coords = results[name]['world_coords']
            error = results[name]['error']
            print(f"  {name}: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f}) - {error}")
    
    return results

def generate_code_output(results):
    '''Generate code snippets for using the results'''
    print("\n" + "=" * 70)
    print("CODE GENERATION")
    print("=" * 70)
    
    # Generate Python dictionary
    print("\n1. PYTHON DICTIONARY (for ur5_sorter.py):")
    print("def get_calculated_joint_angles():")
    print("    '''Pre-calculated joint angles using mathematical IK'''")
    print("    return {")
    
    for name, data in results.items():
        if data['success']:
            angles = data['joint_angles_rad']
            angles_str = "[" + ", ".join([f"{angle:.6f}" for angle in angles]) + "]"
            print(f"        '{name}': {angles_str},")
    
    print("    }")
    
    # Generate usage example
    print("\n2. USAGE EXAMPLE:")
    print("# In your ur5_sorter.py:")
    print("joint_angles_dict = get_calculated_joint_angles()")
    print("ur5.set_joint_angles(joint_angles_dict['home'])")
    print("ur5.set_joint_angles(joint_angles_dict['red_bin'])")
    
    # Generate ROS launch file parameters
    print("\n3. ROS PARAMETERS (for launch files):")
    for name, data in results.items():
        if data['success']:
            angles = data['joint_angles_rad']
            angles_str = "[" + ", ".join([f"{angle:.6f}" for angle in angles]) + "]"
            print(f"    <rosparam param=\"{name}_joint_angles\">{angles_str}</rosparam>")
    
    # Generate YAML format
    print("\n4. YAML FORMAT (for config files):")
    print("ur5_positions:")
    for name, data in results.items():
        if data['success']:
            angles = data['joint_angles_rad']
            print(f"  {name}:")
            print(f"    description: \"{data['description']}\"")
            print(f"    world_coords: {data['world_coords']}")
            print(f"    joint_angles: {angles}")

def save_results_to_files(results):
    '''Save results to various file formats'''
    import json
    
    print("\n" + "=" * 50)
    print("SAVING RESULTS TO FILES")
    print("=" * 50)
    
    # Save to JSON
    try:
        with open('ur5_joint_angles.json', 'w') as f:
            json.dump(results, f, indent=2)
        print("✅ Saved to: ur5_joint_angles.json")
    except Exception as e:
        print(f"❌ Error saving JSON: {e}")
    
    # Save Python code
    try:
        with open('ur5_joint_angles.py', 'w') as f:
            f.write("#!/usr/bin/env python\n")
            f.write("'''\nPre-calculated UR5 joint angles using mathematical IK\n'''\n\n")
            f.write("import math\n\n")
            
            f.write("def get_joint_angles():\n")
            f.write("    '''Return dictionary of pre-calculated joint angles'''\n")
            f.write("    return {\n")
            
            for name, data in results.items():
                if data['success']:
                    angles = data['joint_angles_rad']
                    angles_str = "[" + ", ".join([f"{angle:.6f}" for angle in angles]) + "]"
                    f.write(f"        '{name}': {angles_str},  # {data['description']}\n")
            
            f.write("    }\n\n")
            
            f.write("def get_position_info():\n")
            f.write("    '''Return information about each position'''\n")
            f.write("    return {\n")
            
            for name, data in results.items():
                if data['success']:
                    f.write(f"        '{name}': {{\n")
                    f.write(f"            'description': '{data['description']}',\n")
                    f.write(f"            'world_coords': {data['world_coords']},\n")
                    f.write(f"            'orientation_deg': {data['orientation_deg']}\n")
                    f.write(f"        }},\n")
            
            f.write("    }\n\n")
            
            f.write("# Usage example:\n")
            f.write("# from ur5_joint_angles import get_joint_angles\n")
            f.write("# joint_angles = get_joint_angles()\n")
            f.write("# ur5.set_joint_angles(joint_angles['home'])\n")
        
        print("✅ Saved to: ur5_joint_angles.py")
    except Exception as e:
        print(f"❌ Error saving Python file: {e}")
    
    # Save YAML
    try:
        with open('ur5_joint_angles.yaml', 'w') as f:
            f.write("# UR5 Pre-calculated Joint Angles\n")
            f.write("# Generated using mathematical IK\n\n")
            f.write("ur5_positions:\n")
            
            for name, data in results.items():
                if data['success']:
                    f.write(f"  {name}:\n")
                    f.write(f"    description: \"{data['description']}\"\n")
                    f.write(f"    world_coords: {data['world_coords']}\n")
                    f.write(f"    orientation_deg: {data['orientation_deg']}\n")
                    f.write(f"    joint_angles_rad: {data['joint_angles_rad']}\n")
                    f.write(f"    joint_angles_deg: {data['joint_angles_deg']}\n\n")
        
        print("✅ Saved to: ur5_joint_angles.yaml")
    except Exception as e:
        print(f"❌ Error saving YAML file: {e}")

def interactive_position_tester(results):
    '''Interactive tester for calculated positions'''
    print("\n" + "=" * 50)
    print("INTERACTIVE POSITION TESTER")
    print("=" * 50)
    
    successful_positions = {name: data for name, data in results.items() if data['success']}
    
    if not successful_positions:
        print("❌ No successful positions to test")
        return
    
    print("Available positions:")
    for i, (name, data) in enumerate(successful_positions.items(), 1):
        coords = data['world_coords']
        print(f"  {i:2d}. {name:<20} - {data['description']}")
        print(f"      Coords: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})")
    
    print("\nCommands:")
    print("  <number>  - Show details for position")
    print("  all       - Show all joint angles")
    print("  compare   - Compare two positions")
    print("  quit      - Exit")
    
    position_list = list(successful_positions.items())
    
    while True:
        try:
            cmd = input("\n> ").strip().lower()
            
            if cmd in ['quit', 'q', 'exit']:
                break
            elif cmd == 'all':
                print("\nAll Joint Angles:")
                for name, data in successful_positions.items():
                    angles = data['joint_angles_rad']
                    angles_str = "[" + ", ".join([f"{angle:.4f}" for angle in angles]) + "]"
                    print(f"{name:<20}: {angles_str}")
            elif cmd == 'compare':
                print("Enter two position numbers to compare:")
                try:
                    pos1_idx = int(input("Position 1: ")) - 1
                    pos2_idx = int(input("Position 2: ")) - 1
                    
                    if 0 <= pos1_idx < len(position_list) and 0 <= pos2_idx < len(position_list):
                        name1, data1 = position_list[pos1_idx]
                        name2, data2 = position_list[pos2_idx]
                        
                        print(f"\nComparing {name1} vs {name2}:")
                        print("Joint\t\t{:<20} {:<20} Difference".format(name1[:20], name2[:20]))
                        print("-" * 70)
                        
                        joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
                        for i, joint_name in enumerate(joint_names):
                            angle1 = math.degrees(data1['joint_angles_rad'][i])
                            angle2 = math.degrees(data2['joint_angles_rad'][i])
                            diff = abs(angle1 - angle2)
                            print(f"{joint_name:<12}\t{angle1:>7.2f}°\t\t{angle2:>7.2f}°\t\t{diff:>7.2f}°")
                    else:
                        print("Invalid position numbers")
                except ValueError:
                    print("Please enter valid numbers")
            elif cmd.isdigit():
                idx = int(cmd) - 1
                if 0 <= idx < len(position_list):
                    name, data = position_list[idx]
                    coords = data['world_coords']
                    orient = data['orientation_deg']
                    
                    print(f"\n--- {name.upper().replace('_', ' ')} ---")
                    print(f"Description: {data['description']}")
                    print(f"World coordinates: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})")
                    print(f"Orientation: roll={orient[0]}°, pitch={orient[1]}°, yaw={orient[2]}°")
                    print("\nJoint Angles:")
                    
                    joint_names = ['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 
                                  'ur5_elbow_joint', 'ur5_wrist_1_joint', 
                                  'ur5_wrist_2_joint', 'ur5_wrist_3_joint']
                    
                    for joint_name, angle_rad in zip(joint_names, data['joint_angles_rad']):
                        angle_deg = math.degrees(angle_rad)
                        print(f"  {joint_name:<25}: {angle_deg:>7.2f}° ({angle_rad:>8.4f} rad)")
                    
                    print(f"\nFor ur5.set_joint_angles():")
                    angles_str = "[" + ", ".join([f"{angle:.4f}" for angle in data['joint_angles_rad']]) + "]"
                    print(f"  {angles_str}")
                else:
                    print("Invalid position number")
            else:
                print("Unknown command. Type 'quit' to exit.")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

def main():
    '''Main function'''
    print("Starting UR5 batch position calculation...")
    
    # Calculate all positions
    results = calculate_all_positions()
    
    if not results:
        print("❌ Failed to calculate positions")
        return
    
    # Generate code output
    generate_code_output(results)
    
    # Save to files
    save_results_to_files(results)
    
    # Interactive tester
    response = input("\nWould you like to test positions interactively? (y/n): ")
    if response.lower() == 'y':
        interactive_position_tester(results)
    
    print("\n🎉 Batch processing complete!")
    print("Check the generated files:")
    print("  - ur5_joint_angles.json (data)")
    print("  - ur5_joint_angles.py (Python code)")
    print("  - ur5_joint_angles.yaml (YAML config)")

if __name__ == "__main__":
    main()