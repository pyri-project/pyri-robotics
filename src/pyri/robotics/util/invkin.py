import numpy as np
import general_robotics_toolbox as rox
from scipy.optimize import lsq_linear

def update_ik_info3(robot_rox, T_desired, q_current): # inverse kinematics that uses Least Square solver
    
    # R_d, p_d: Desired orientation and position
    R_d = T_desired.R
    p_d = T_desired.p
    d_q = q_current

    num_joints = len(robot_rox.joint_type)
    
    q_cur = d_q # initial guess on the current joint angles
    q_cur = q_cur.reshape((num_joints,1)) 
    
    max_steps = 200 # number of steps to for convergence
    
    # print_div( "<br> q_cur " + str(q_cur) ) # DEBUG

    hist_b = []
    
    itr = 0 # Iterations
    converged = False
    while itr < max_steps and not converged:
        
        pose = rox.fwdkin(robot_rox,q_cur)
        R_cur = pose.R
        p_cur = pose.p
        
        #calculate current Jacobian
        J0T = rox.robotjacobian(robot_rox,q_cur)
        
        # Transform Jacobian to End effector frame from the base frame
        Tr = np.zeros((6,6))
        Tr[:3,:3] = R_cur.T 
        Tr[3:,3:] = R_cur.T
        J0T = Tr @ J0T
        
        # Jp=J0T[3:,:]
        # JR=J0T[:3,:]                      #decompose to position and orientation Jacobian
        
        # Error in position and orientation
        # ER = np.matmul(R_cur, np.transpose(R_d))
        ER = np.matmul(np.transpose(R_d),R_cur)
        #print_div( "<br> ER " + str(ER) ) # DEBUG

        # EP = p_cur - p_d                         
        EP = R_cur.T @ (p_cur - p_d)                         
        #print_div( "<br> EP " + str(EP) ) # DEBUG

        #decompose ER to (k,theta) pair
        k, theta = rox.R2rot(ER)                  
        # print_div( "<br> k " + str(k) ) # DEBUG
        # print_div( "<br> theta " + str(theta) ) # DEBUG
        
        ## set up s for different norm for ER
        # s=2*np.dot(k,np.sin(theta)) #eR1
        # s = np.dot(k,np.sin(theta/2))         #eR2
        s = np.sin(theta/2) * np.array(k)         #eR2
        # s=2*theta*k              #eR3
        # s=np.dot(J_phi,phi)              #eR4
        # print_div( "<br> s " + str(s) ) # DEBUG         

        Kp = np.eye(3)
        KR = np.eye(3)        #gains for position and orientation error
        
        vd = - Kp @ EP
        wd = - KR @ s
        
        b = np.concatenate([wd,vd])
        np.nan_to_num(b, copy=False, nan=0.0, posinf=None, neginf=None)
        # print(b)
        # print(J0T)
        
        # DEBUG --------------
        hist_b.append(b)
        if itr > 0:
            error_cur = np.linalg.norm(hist_b[itr-1]) - np.linalg.norm(hist_b[itr])
            print("Error= " + str(error_cur))
        # DEBUG --------------

        res = lsq_linear(J0T,b)

        if res.success: 
            qdot_star = res.x 
        else:
            print("Any solution could not found")
            qdot_star = np.finfo(float).eps * np.ones(num_joints)

        # find best step size to take
        # alpha=fminbound(min_alpha,0,1,args=(q_cur,qdot_star,Sawyer_def,Rd,pd,w,Kp))
        alpha = 0.2 # Step size    # 1.0    
        delta = alpha * qdot_star 
        # print_div( "<br> delta " + str(delta) ) # DEBUG
                    
        # Convergence Check
        converged = (np.abs(np.hstack((s,EP))) < 0.0001).all()

        if not converged:
            # Update for next iteration
            q_cur = q_cur + delta.reshape((num_joints,1))

            # Normalize angles betweeen -pi to pi
            q_cur = normalizeAngles(q_cur)
        
        # print_div( "<br> converged? " + str(converged) ) # DEBUG
        # print( "converged? " + str(converged) ) # DEBUG
        
        itr += 1 # Increase the iteration
        print(itr)
        print(converged)
        # print(delta)
        # print(q_cur)
    
    # joints_text=""
    # for i in q_cur:
    #     joints_text+= "(%.3f, %.3f) " % (np.rad2deg(i), i)   
    # print_div_ik_info(str(rox.Transform(R_d,p_d)) +"<br>"+ joints_text +"<br>"+ str(converged) + ", itr = " + str(itr))
    return np.squeeze(q_cur), converged

def normalizeAngle(angle):
    """
    :param angle: (float)
    :return: (float) the angle in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle 

def normalizeAngles(angles):
    for idx, angle in np.ndenumerate(angles):
        angles[idx] = normalizeAngle(angle)

    return angles