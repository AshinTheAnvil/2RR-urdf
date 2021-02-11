
import pybullet as p
import pybullet_data as pd
import numpy as np
from numpy import *
import time


class two_dof_rr:
    ## constructor and attributes
    def __init__(self):
     
      self.rob_id = None
      self.num_jts = None
      self.mvbl_jts = None
      
      self.end_eff_index = 0

      self.time_step= 1/240
     


      ## setting up pybullet engine
    def createWorld(self):
        
       
        pc = p.connect(p.GUI)
        
        p.setAdditionalSearchPath(pd.getDataPath())
        GRAVITY = -10
        p.setGravity(0, 0, GRAVITY)
        plane_id = p.loadURDF("plane.urdf")
        
        
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter()
        p.setRealTimeSimulation(1)
        
        
        ### 2link cuiboidal arm is being loaded into the environment
        
        self.rob_id = p.loadURDF('urdf/2link.urdf', useFixedBase = 1)
        self.num_jts = p.getNumJoints(self.rob_id) # Joints(including floor and base link which is a fixed one..)
        print('Number of Joints in my arm: ',self.num_jts)
        
        ##here the joints is taken n-1 since base has a fixed joint,hence movable will be n-1
        self.mvbl_jts = list(range(1, self.num_jts - 1))
        print('movable joints:', self.mvbl_jts)
        self.end_eff_index = self.mvbl_jts[-1]
        print('#End-effector:', self.end_eff_index)
        
        
        
        
    def getJointStates(self):
        joint_states = p.getJointStates(self.rob_id, self.mvbl_jts)
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques
        
    def getJacobian(self, joint_pos):
        eestate = p.getLinkState(self.rob_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eestate
        zero_vec = [0.0] * len(joint_pos)
        jac_trans, jac_rot = p.calculateJacobian(self.rob_id, self.end_eff_index, com_trn, list(joint_pos), zero_vec, zero_vec)
        j_t = np.asarray(jac_trans)
        j_r = np.asarray(jac_rot)
        j = np.concatenate((j_t, j_r))
        print('Jacobian:', j)
        
        return j
        
    def solveForwardPositonKinematics(self, joint_pos):
        print('Forward position kinematics')

        # get end-effector link state
        eeState = p.getLinkState(self.rob_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
        eePose = list(link_trn) + list(p.getEulerFromQuaternion(link_rot))
        print('End-effector pose:', eePose)
        return eePose
        
    def solveForwardVelocityKinematics(self, joint_pos, joint_vel):
        
        J  = self.getJacobian(joint_pos)
        eeVelocity = J @ joint_vel
        
        return eeVelocity

        
    def calculateDynamicMatrices(self):
        joint_pos, joint_vel, acc = self.getJointStates()
        n_dof = len(self.mvbl_jts)
        InertiaMatrix= np.asarray(p.calculateMassMatrix(self.rob_id, joint_pos))
        GravityMatrix = np.asarray(p.calculateInverseDynamics(self.rob_id, joint_pos, [0.0] * n_dof, [0.0] * n_dof))
        CoriolisMatrix = np.asarray(p.calculateInverseDynamics(self.rob_id, joint_pos, joint_vel, [0.0] * n_dof)) - GravityMatrix
        return InertiaMatrix, GravityMatrix, CoriolisMatrix
        
        
    def tk_cmd(self,rest,rangeMin = -6.28,rangeMax = 6.28):

        x = p.addUserDebugParameter("x", rangeMin, rangeMax, rest[0])
        y = p.addUserDebugParameter("y", rangeMin, rangeMax, rest[1]) #y
        z = p.addUserDebugParameter("z", rangeMin, rangeMax, rest[2]) #z
        r = p.addUserDebugParameter("r", rangeMin, rangeMax, rest[3]) #roll
        pit = p.addUserDebugParameter("p",rangeMin, rangeMax, rest[4]) #pitch
        y = p.addUserDebugParameter("y", rangeMin, rangeMax, rest[5]) # yaw
        return [x, y, z, r, pit, y]

    def contact_forces(self,forces, max_limit = 1.0, min_limit = -1.0):
        fx = p.addUserDebugParameter("f_in_x", min_limit, max_limit, forces[0]) #force along x
        fy = p.addUserDebugParameter("f_in_y", min_limit, max_limit, forces[1]) #force along y
        fz = p.addUserDebugParameter("f_in_z", min_limit, max_limit, forces[2]) #force along z
        mx = p.addUserDebugParameter("m_in_x", min_limit, max_limit, forces[3]) #moment along x
        my = p.addUserDebugParameter("m_in_y", min_limit, max_limit, forces[4]) #moment along y
        mz = p.addUserDebugParameter("m_in_z", min_limit, max_limit, forces[5]) #moment along z
        return [fx, fy, fz, mx, my, mz]

    # function to read the value of task parameter
    def gui_saction(self, ids):
        val1 = p.readUserDebugParameter(ids[0])
        val2 = p.readUserDebugParameter(ids[1])
        val3 = p.readUserDebugParameter(ids[2])
        val4 = p.readUserDebugParameter(ids[3])
        val5 = p.readUserDebugParameter(ids[4])
        val6 = p.readUserDebugParameter(ids[5])
        return np.array([val1, val2, val3, val4, val5, val6])
            
    
    def imp_con(self):
        p.setRealTimeSimulation(False)
        for lid in range(self.num_jts+1):
            p.changeDynamics(self.rob_id, lid, linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
            
        
        p.setJointMotorControlArray(self.rob_id, self.mvbl_jts,
                                    p.VELOCITY_CONTROL,
                                    forces=np.zeros(len(self.mvbl_jts)))
        
        xd = [0,0,0,0,0,0]
        dxd = [0,0,0,0,0,0]
        pos_gui = self.tk_cmd(xd)
        fi = [0,0,0,0,0,0]
        force_gui = self.contact_forces(fi)
        # stiffness and damping constant
        
        Kp = 500
        Kd = 10
        # Mm is a desired inertia based matrix which is mostly diagonal
        # Source : http://www.dis.uniroma1.it/~deluca/rob2_en/15_ImpedanceControl.pdf
        Mm = 0.01*np.eye(6)
    
        
        while True:
            ## practically external force is measured using 6 axis force sensor which measures force applied to end effector also sensorless force control is also possible
            ## F_ext is forces and moments in x y z directions giving a wrench
           
            xd = self.gui_saction(pos_gui) 
            F_ext = self.gui_saction(force_gui) 

           
            q, dq, _ = self.getJointStates()

            # Error in task space
            x = self.solveForwardPositonKinematics(q)
            x_e = xd - x
            dx = self.solveForwardVelocityKinematics(q, dq)
            dx_e = dxd - dx
            
            J = self.getJacobian(q)
            J_inv = np.linalg.pinv(J)
            Mq, G, _ = self.calculateDynamicMatrices()
            print("Mq",Mq)
            #Mm = Mq
            
            Mx = np.dot(np.dot(np.transpose(J_inv), Mq), J_inv)
            
            #Fx = np.dot(np.dot(np.linalg.inv(Mm), Mx),(np.dot(Kp, x_e) + np.dot(Kd, dx_e)))
            #print("1111",Fx)
            #F_w_ext = np.dot((np.dot(np.linalg.inv(Mm), Mx) - np.eye(6)), F_ext)
            #print("2222",F_w_ext)
            #Fx += F_w_ext
            I = np.eye(6)
            f1 = np.dot(np.dot(np.linalg.pinv(Mm), Mx),(np.dot(Kp, x_e) + np.dot(Kd, dx_e)))
            f2 = np.dot((np.dot(np.linalg.inv(Mm), Mx) - I), F_ext)
            #ff =
            #f3 = f1 + f2
            #these values are again converted to joint space for implementation of torque to joints
            f11 = np.dot(np.transpose(J),f1)
            f22 = np.dot(np.transpose(J),f2)
            f33 = f11+f22+G
        
            
            #t = np.dot(np.transpose(J),f3) + G
            print("%%%% final torque",f33)
            #torque = G + f_jt
            #print("$$$$$",torque)
            t = f33
            p.setJointMotorControlArray(self.rob_id, self.mvbl_jts,
                                        controlMode = p.TORQUE_CONTROL,
                                        forces = t)

            p.stepSimulation()
            time.sleep(self.time_step)
        p.disconnect()


        ### calling my def and other computations
if __name__ == '__main__':

    robo = two_dof_rr()
    robo.createWorld()
    
    robo.imp_con()
    
   

   
