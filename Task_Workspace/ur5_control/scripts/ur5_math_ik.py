#!/usr/bin/env python
'''
Corrected UR5 Mathematical Inverse Kinematics Module
Accounts for robot base position offset in world coordinates

Your robot base is at (0, 0, 0.95) in world coordinates, not (0, 0, 0)
'''

import math

class UR5MathIK:
    '''
    Mathematical IK solver accounting for robot base position in world coordinates.
    '''
    
    def __init__(self):
        '''Initialize with UR5 DH parameters and world offset'''
        # UR5 DH parameters (robot-relative)
        self.d1 = 0.089159    # Base height (relative to robot base)
        self.a2 = 0.425       # Upper arm length  
        self.a3 = 0.39225     # Forearm length
        self.d4 = 0.10915     # Wrist 1 offset
        self.d5 = 0.09465     # Wrist 2 offset
        self.d6 = 0.0823      # Wrist 3 to end effector
        
        # IMPORTANT: Robot base position in world coordinates
        # From your URDF: <origin xyz="0.0 0.0 0.95"/>
        self.robot_base_world_x = 0.0
        self.robot_base_world_y = 0.0
        self.robot_base_world_z = 0.95
        
        # Joint limits (radians)
        self.joint_limits = [
            (-math.pi, math.pi),      # Joint 1 (base)
            (-math.pi, math.pi),      # Joint 2 (shoulder)
            (-math.pi, math.pi),      # Joint 3 (elbow)
            (-math.pi, math.pi),      # Joint 4 (wrist 1)
            (-math.pi, math.pi),      # Joint 5 (wrist 2)
            (-math.pi, math.pi)       # Joint 6 (wrist 3)
        ]
    
    def world_to_robot_coords(self, world_x, world_y, world_z):
        '''
        Convert world coordinates to robot base coordinates.
        
        Args:
            world_x, world_y, world_z: Position in world frame
            
        Returns:
            tuple: (robot_x, robot_y, robot_z) relative to robot base
        '''
        robot_x = world_x - self.robot_base_world_x
        robot_y = world_y - self.robot_base_world_y
        robot_z = world_z - self.robot_base_world_z
        
        return robot_x, robot_y, robot_z
    
    def robot_to_world_coords(self, robot_x, robot_y, robot_z):
        '''
        Convert robot base coordinates to world coordinates.
        
        Args:
            robot_x, robot_y, robot_z: Position relative to robot base
            
        Returns:
            tuple: (world_x, world_y, world_z) in world frame
        '''
        world_x = robot_x + self.robot_base_world_x
        world_y = robot_y + self.robot_base_world_y
        world_z = robot_z + self.robot_base_world_z
        
        return world_x, world_y, world_z
    
    def solve(self, world_x, world_y, world_z, roll=0, pitch=math.pi/2, yaw=0):
        '''
        Solve IK for world coordinates.
        
        Args:
            world_x, world_y, world_z: Target position in world frame
            roll, pitch, yaw: Target orientation in radians
        
        Returns:
            list: Joint angles [j1, j2, j3, j4, j5, j6] in radians or None
        '''
        try:
            # Convert world coordinates to robot base coordinates
            x, y, z = self.world_to_robot_coords(world_x, world_y, world_z)
            
            print(f"World coords: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
            print(f"Robot coords: ({x:.3f}, {y:.3f}, {z:.3f})")
            
            # Joint 1: Base rotation around Z-axis
            theta1 = math.atan2(y, x)
            
            # Calculate 2D problem in the arm plane
            r = math.sqrt(x**2 + y**2) - self.d4  # Horizontal distance minus wrist offset
            z_eff = z - self.d1 - self.d6  # Effective height (subtract base height and tool length)
            
            # Distance from shoulder to wrist center
            d = math.sqrt(r**2 + z_eff**2)
            
            print(f"Effective coords: r={r:.3f}, z_eff={z_eff:.3f}, d={d:.3f}")
            
            # Check reachability
            max_reach = self.a2 + self.a3
            min_reach = abs(self.a2 - self.a3)
            
            if d > max_reach:
                print(f"❌ Target too far: {d:.3f} > {max_reach:.3f}")
                return None
            if d < min_reach:
                print(f"❌ Target too close: {d:.3f} < {min_reach:.3f}")
                return None
            
            print("✅ Target is reachable")
            
            # Joint 3: Elbow angle using cosine rule
            cos_theta3 = (d**2 - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
            
            if abs(cos_theta3) > 1.0:
                print("❌ Elbow calculation failed")
                return None
            
            # Elbow up configuration (positive angle)
            sin_theta3 = math.sqrt(1 - cos_theta3**2)
            theta3 = math.atan2(sin_theta3, cos_theta3)
            
            # Joint 2: Shoulder angle
            alpha = math.atan2(z_eff, r)
            beta = math.atan2(self.a3 * sin_theta3, self.a2 + self.a3 * cos_theta3)
            theta2 = alpha - beta
            
            # Joint 4: Wrist 1 angle (keep end effector level)
            theta4 = -theta2 - theta3
            
            # Joint 5: Wrist 2 angle (90° for downward pointing)
            theta5 = math.pi/2
            
            # Joint 6: Wrist 3 angle (tool rotation)
            theta6 = yaw - theta1
            
            # Normalize all angles to [-pi, pi]
            joints = [theta1, theta2, theta3, theta4, theta5, theta6]
            joints = [self._normalize_angle(angle) for angle in joints]
            
            # Check joint limits
            if not self._check_limits(joints):
                print("⚠ Solution exceeds joint limits")
                return None
            
            print("✅ IK calculation successful!")
            return joints
            
        except Exception as e:
            print(f"❌ IK calculation error: {e}")
            return None
    
    def check_reachability(self, world_x, world_y, world_z):
        '''
        Check if world position is reachable.
        
        Args:
            world_x, world_y, world_z: Position in world frame
        
        Returns:
            bool: True if position is likely reachable
        '''
        # Convert to robot coordinates
        x, y, z = self.world_to_robot_coords(world_x, world_y, world_z)
        
        # Horizontal distance from robot base
        horizontal_dist = math.sqrt(x**2 + y**2)
        max_horizontal = self.a2 + self.a3
        min_horizontal = abs(self.a2 - self.a3)
        
        if horizontal_dist > max_horizontal or horizontal_dist < min_horizontal:
            return False
        
        # Height limits (relative to robot base)
        max_height = self.d1 + self.a2 + self.a3 + self.d6
        min_height = self.d1 - self.a2 - self.a3 + self.d6
        
        return min_height <= z <= max_height
    
    def get_workspace_info(self):
        '''Get robot workspace information in world coordinates'''
        # Robot-relative limits
        max_horizontal = self.a2 + self.a3
        min_horizontal = abs(self.a2 - self.a3)
        max_height_robot = self.d1 + self.a2 + self.a3 + self.d6
        min_height_robot = self.d1 - self.a2 - self.a3 + self.d6
        
        # Convert to world coordinates
        max_height_world = max_height_robot + self.robot_base_world_z
        min_height_world = min_height_robot + self.robot_base_world_z
        
        return {
            'max_horizontal_reach': max_horizontal,
            'min_horizontal_reach': min_horizontal,
            'max_height_world': max_height_world,
            'min_height_world': min_height_world,
            'robot_base_world_z': self.robot_base_world_z,
            'robot_base_height': self.d1,
            'tool_length': self.d6
        }
    
    def _normalize_angle(self, angle):
        '''Normalize angle to [-pi, pi] range'''
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _check_limits(self, joint_angles):
        '''Check if joint angles are within limits'''
        for i, (angle, (lower, upper)) in enumerate(zip(joint_angles, self.joint_limits)):
            if angle < lower or angle > upper:
                return False
        return True

# Convenience functions for easy use
def calculate_joint_angles(world_x, world_y, world_z, roll=0, pitch=math.pi/2, yaw=0):
    '''
    Convenience function to calculate joint angles from world coordinates.
    
    Args:
        world_x, world_y, world_z: Target position in world frame
        roll, pitch, yaw: Target orientation in radians
    
    Returns:
        list: Joint angles in radians or None if no solution
    '''
    solver = UR5MathIK()
    return solver.solve(world_x, world_y, world_z, roll, pitch, yaw)

def is_position_reachable(world_x, world_y, world_z):
    '''
    Quick check if world position is reachable.
    
    Returns:
        bool: True if position is within workspace
    '''
    solver = UR5MathIK()
    return solver.check_reachability(world_x, world_y, world_z)

def get_ur5_workspace():
    '''Get UR5 workspace limits in world coordinates'''
    solver = UR5MathIK()
    return solver.get_workspace_info()

def test_with_your_pose():
    '''Test with the pose values you provided'''
    print("=== Testing with Your Pose Values ===")
    
    # Your pose values
    world_x = -0.000003
    world_y = 0.000014  
    world_z = 1.039159
    roll = -0.000054
    pitch = 1.569283
    yaw = -1.570768
    
    print(f"Input world coordinates: ({world_x:.6f}, {world_y:.6f}, {world_z:.6f})")
    print(f"Input orientation (rad): roll={roll:.6f}, pitch={pitch:.6f}, yaw={yaw:.6f}")
    print(f"Input orientation (deg): roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
    
    solver = UR5MathIK()
    
    # Check reachability first
    reachable = solver.check_reachability(world_x, world_y, world_z)
    print(f"Position reachable: {reachable}")
    
    # Calculate joint angles
    joint_angles = solver.solve(world_x, world_y, world_z, roll, pitch, yaw)
    
    if joint_angles:
        print("\n✅ IK Solution Found!")
        joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
        
        print("Joint angles (degrees):")
        for name, angle in zip(joint_names, joint_angles):
            print(f"  {name}: {math.degrees(angle):.2f}°")
        
        print("\nJoint angles (radians) for set_joint_angles():")
        for name, angle in zip(joint_names, joint_angles):
            print(f"  {name}: {angle:.6f}")
        
        print("\nAs list:")
        angles_str = "[" + ", ".join([f"{angle:.6f}" for angle in joint_angles]) + "]"
        print(f"  {angles_str}")
    else:
        print("\n❌ No IK solution found")
    
    # Show workspace info
    workspace = solver.get_workspace_info()
    print(f"\n=== Workspace Info ===")
    print(f"Robot base in world: (0, 0, {workspace['robot_base_world_z']:.3f})")
    print(f"Horizontal reach: {workspace['min_horizontal_reach']:.3f} - {workspace['max_horizontal_reach']:.3f} m")
    print(f"World height range: {workspace['min_height_world']:.3f} - {workspace['max_height_world']:.3f} m")

if __name__ == "__main__":
    test_with_your_pose()