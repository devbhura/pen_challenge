from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np




def main():
    bot = InterbotixManipulatorXS("px100", "arm", "gripper")
    # bot.arm.go_to_sleep_pose()
 
    bot.gripper.open()
    bot.arm.set_ee_pose_components(x=0.25,y = 0.14, z=0.3)
    

    bot.gripper.close()
    
    # bot.arm.set_ee_cartesian_trajectory(x=0.05, z=-0.05)
    # bot.gripper.close()
    # bot.arm.set_ee_cartesian_trajectory(x=-0.05, z=0.05)
    # bot.arm.set_single_joint_position("waist", -np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    # bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    # bot.arm.set_single_joint_position("shoulder", np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(x=0.05, z=-0.05)
    # bot.gripper.open()
    # bot.arm.set_ee_cartesian_trajectory(x=-0.05, z=0.05)
    # bot.arm.go_to_home_pose()
    # bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
