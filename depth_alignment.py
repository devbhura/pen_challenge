# import packages 
from numpy.lib.function_base import average
import pyrealsense2 as rs
import numpy as np 
import cv2
from interbotix_xs_modules.arm import InterbotixManipulatorXS

def move(coor):
    bot = InterbotixManipulatorXS("px100", "arm", "gripper")
    
    bot.arm.set_ee_pose_components(x=round((0.20 - coor[0]),3), y=round((0.235 - coor[2]),3), z=round((0.15 - coor[1]),3))
    bot.gripper.close()
    bot.arm.go_to_home_pose()
    bot.arm.set_single_joint_position("waist", -np.pi/2.0)
    bot.gripper.open()
    bot.arm.go_to_sleep_pose()
    


pipeline = rs.pipeline()
config = rs.config()
config.enable_record_to_file("Data")
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
pro = profile.get_stream(rs.stream.color)
intr = pro.as_video_stream_profile().get_intrinsics()


# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


cv2.namedWindow('Control')
cv2.createTrackbar('LowH','Control', 30, 180, lambda x:x)
cv2.createTrackbar('HighH','Control', 160, 180, lambda x:x)
cv2.createTrackbar('LowS','Control', 60, 255, lambda x:x)
cv2.createTrackbar('HighS','Control', 240, 255, lambda x:x)
cv2.createTrackbar('LowV','Control', 28, 255, lambda x:x)
cv2.createTrackbar('HighV','Control', 240, 255, lambda x:x)

coor_x = []
coor_y = []
coor_z = []
# Streaming loop
try:
    while True:

        
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)


        # convert rgb to hsv
        hsv = cv2.cvtColor(bg_removed,cv2.COLOR_BGR2HSV)

        lowH = cv2.getTrackbarPos('LowH','Control')
        lowS = cv2.getTrackbarPos('LowS','Control')
        lowV = cv2.getTrackbarPos('LowV','Control')
        highH = cv2.getTrackbarPos('HighH','Control')
        highS = cv2.getTrackbarPos('HighS','Control')
        highV = cv2.getTrackbarPos('HighV','Control')

        lower_t = np.array([lowH,lowS,lowV])
        upper_t = np.array([highH,highS,highV])
        mask = cv2.inRange(hsv,lower_t,upper_t)

        

        res = cv2.bitwise_and(bg_removed,bg_removed,mask = mask)
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 2:
            cnt = max(contours, key  = cv2.contourArea)
            
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            depth = depth_scale*(depth_image[cy][cx])
            
            centroid = [cx,cy]
            print(depth)
            coor = np.array(rs.rs2_deproject_pixel_to_point(intr,centroid,depth))
            cv2.circle(bg_removed,[cx,cy],2,[255,0,0],6)
            
            
            coor_x.append(coor[0])
            coor_y.append(coor[1])
            coor_z.append(coor[2])

            if len(coor_x)>20:
                coor_x.pop(0)
                coor_y.pop(0)
                coor_z.pop(0)  

            avgx = average(coor_x)
            avgy = average(coor_y)
            avgz = average(coor_z)
            coor_avg = [avgx,avgy,avgz]
            print(coor_avg)

        
        cv2.drawContours(res,contours,-1,[0, 225, 0])
        
        cv2.imshow('Res', res)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

        
finally:
    pipeline.stop() 

move(coor_avg)
print(coor_avg)
