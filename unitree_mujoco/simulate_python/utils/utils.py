import numpy as np
import mujoco as mj
from math import atan,atan2, pi, tan


'''
Gets the Camera Matrix
'''
def get_camera_matrix(model, cam_id, W, H):
    fovy = model.cam_fovy[cam_id]
    aspect_ratio = W/H
    fovx = 2 * atan(tan(fovy * pi / 360) * aspect_ratio) * 180 / pi
    
    #focal lengths
    
    fy = 0.5 * H / tan(fovy * pi / 360)
    fx = 0.5 * W / tan(fovx * pi / 360)

    #principle points - centre of the frame
    cx = W / 2.0
    cy = H / 2.0

    camera_matrix = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]])
    
    return camera_matrix
'''
Camera pixel to world frame
'''
def camera_to_world(camera_matrix,u,v,depth):
    fx,fy = camera_matrix[0][0], camera_matrix[1][1]
    cx,cy = camera_matrix[0][2], camera_matrix[1][2]
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return np.array([x, y, z])
'''
Parameters required for transformation between world and camera
'''

def get_camera_extrinsics(model, data, cam_id):
    
    body_id = model.cam_bodyid[cam_id]

    #camera relative positions and orienatation w.r.t body frame - REMAIN FIXED
    cam_pos = model.cam_pos[cam_id]
    cam_mat = model.cam_mat0[cam_id].reshape(3,3)
    
    #camera body postion and orientation wrt world - VARIABLE
    body_pos = data.xpos[body_id]
    body_rot = data.xmat[body_id].reshape(3,3)

    #transformation
    R = body_rot @ cam_mat
    t = body_pos + body_rot @ cam_pos

    return R,t

'''
Get Transform between camera frame and any other body - end effector, base etc..
'''
def getpose_relative2body(model,data,name,R,t,Xc):
    # pose relative to world from camera
    Xw = R@Xc + t
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
    pos = data.xpos[body_id]
    rot = data.xmat[body_id].reshape(3,3)

    #rot is supposed to be orthogonal hence, its inverse is equal to transpose
    X_rel = np.transpose(rot)@(Xw - pos)
    return X_rel



    



    



