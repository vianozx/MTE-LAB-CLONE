#!/usr/bin/env python

import rospy
from math import radians
from lib import UR5MoveIt

def test_joint_tolerance():
    """Test script to check joint movement and tolerance"""
    
    ur5 = UR5MoveIt()
    
    # Home position from your script
    home_joint_angles = [radians(0),
                         radians(-90),
                         radians(-90),
                         radians(-90),
                         radians(90),
                         radians(0)]
    
    rospy.loginfo("Testing joint movement to home position...")
    
    # Get current joint values
    current = ur5._group.get_current_joint_values()
    rospy.loginfo(f"Current joints: {[round(j, 4) for j in current]}")
    rospy.loginfo(f"Target joints:  {[round(j, 4) for j in home_joint_angles]}")
    
    # Try to move
    result = ur5.set_joint_angles(home_joint_angles)
    
    # Check final position
    final = ur5._group.get_current_joint_values()
    rospy.loginfo(f"Final joints:   {[round(j, 4) for j in final]}")
    
    # Calculate errors
    errors = [abs(f - t) for f, t in zip(final, home_joint_angles)]
    rospy.loginfo(f"Joint errors:   {[round(e, 6) for e in errors]}")
    
    max_error = max(errors)
    rospy.loginfo(f"Maximum error: {max_error:.6f} radians")
    
    if max_error < 0.01:
        rospy.loginfo("SUCCESS: All joints within 0.01 rad tolerance!")
    else:
        rospy.logwarn(f"ISSUE: Some joints exceed tolerance. Max error: {max_error:.6f}")

if __name__ == "__main__":
    test_joint_tolerance()