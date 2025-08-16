import mujoco as mj
from mujoco.glfw import glfw
from OpenGL.GL import *
import os
import numpy as np
# np.set_printoptions(threshold=np.inf)

import matplotlib.pyplot as plt

from utils.detect_and_draw_bound import detect_and_draw_bound
from utils.get_frame import get_frame
from unitree_sdk2py_bridge import UnitreeSdk2Bridge
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from  utils import utils

import config
import cv2
print_camera_config = 0  # set to 1 to print camera config
need_heatmap = False
show_frame = True

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# inset screen width and height
frame_width = 1200
frame_height = 720
dx = 0  # frame center xpos - bounding box center xpos
dy = 0  # frame center ypos - bounding box center ypos
bounding_box = np.array([0, 0, 0, 0])  # [x, y, x+w, y+h]: bottom left and top right coordinates


"""
Convert a 1D depth array (in meters) into a heatmap image.
"""
def depth_to_colormap(depth_flat, width=1200, height=720):
    depth_map = depth_flat.reshape((height, width))

    depth_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    depth_norm = depth_norm.astype(np.uint8)

    heatmap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
    heatmap = cv2.rotate(heatmap, cv2.ROTATE_180)

    return heatmap

'''
Contains keyboard and mouse callbacks for OpenGL
'''
    
def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)


def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mm.jMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)


def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

'''
Initialise Communication with Cyclone DDS for Unitree
'''

ChannelFactoryInitialize(0)

'''
Mujoco Parameters
'''
xml_path = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),'unitree_robots/g1/scene_23dof.xml')
model = mj.MjModel.from_xml_path(xml_path)  
model.opt.timestep = 0.002# MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options
cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, 'robot_camera')

camera_matrix = utils.get_camera_matrix(model,cam_id,640,480)

extent = model.stat.extent
znear = model.vis.map.znear
zfar = model.vis.map.zfar


glfw.init()
window = glfw.create_window(6400, 6400, "OpenCV Tracking", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)

scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)



cam.azimuth = 81 ; cam.elevation = -25 ; cam.distance =  9
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])


unitree = UnitreeSdk2Bridge(model,data)
print(unitree.PrintSceneInformation())

while not glfw.window_should_close(window):
    time_prev = data.time
    

    while (data.time - time_prev < 0.001):
        mj.mj_step(model, data)


    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    
    frame, offscreen_viewport,depth = get_frame(model, data, opt, scene, context, 'robot_camera', loc_x=viewport_width - frame_width, loc_y=viewport_height - frame_height, width=frame_width, height=frame_height)
    frame_re = frame.reshape((frame_height, frame_width, 3))
    frame_boundbox, bounding_box = detect_and_draw_bound(frame, width=frame_width, height=frame_height)
    

    depth_m = znear/ (1 - depth * (1 - znear / zfar))
    depth_re = depth_m.reshape((1200,720))
    
    #centre of the bounding box
    u = int((bounding_box['red']['bbox'][0] + bounding_box['red']['bbox'][2])/2)
    v = int((bounding_box['red']['bbox'][1]+ bounding_box['red']['bbox'][3])/2)
    Xc = utils.camera_to_world(camera_matrix,u,v,depth_re[u][v])
    print("depth of the centre",depth_re[u][v])
    print("location wrt camera of the object",Xc)

    #relative to world from camera

    R,t = utils.get_camera_extrinsics(model,data,cam_id) 
    # wrt to another link 
    print("locations wrt right wrist", utils.getpose_relative2body(model,data,"right_wrist_roll_rubber_hand",R,t,Xc) )
    
    if show_frame:
        cv2.rectangle(frame_re, (bounding_box['red']['bbox'][0], bounding_box['red']['bbox'][1]), (bounding_box['red']['bbox'][2], bounding_box['red']['bbox'][3]), (0, 255, 0), 2)
        cv2.circle(frame_re, (u,v), 3, (255,255,0), -1)
        text = "real-time position" + str(Xc)
        frame_re = cv2.rotate(frame_re, cv2.ROTATE_180)
        # origin = (frame_height-1-u,frame_width-1-v)
        origin = (150,150)
        cv2.putText(frame_re, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 2)
        cv2.imshow("Frame", frame_re)

        if(cv2.waitKey(25) & 0xFF) == ord('q'):
            cv2.destroyAllWindows()
            break

    
    
    if need_heatmap:
        heatmap = depth_to_colormap(depth_m)
        cv2.circle(heatmap, (u,v), 3, (255,255,0), -1)

        cv2.imshow("Depth Heatmap", heatmap)
        if (cv2.waitKey(25) & 0xFF) == ord('q'):
            cv2.destroyAllWindows()
            break

    glClear(GL_DEPTH_BUFFER_BIT) 
    mj.mjr_drawPixels(frame_boundbox, None, offscreen_viewport, context)
    
    glfw.swap_buffers(window)

    glfw.poll_events()

glfw.terminate()
