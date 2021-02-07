

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
      
      self.end_eff_index = 2

      self.time_step= 1/240
     


      ## setting up pybullet engine
    def createWorld(self):
        
       
        pc = p.connect(p.GUI)
        
        p.setAdditionalSearchPath(pd.getDataPath())
        GRAVITY = -9.8
        p.setGravity(0, 0, GRAVITY)
        plane_id = p.loadURDF("plane.urdf")
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter()
        p.setRealTimeSimulation(1)
        
        
        ### 2link cuiboidal arm is being loaded into the environment
        
        self.rob_id = p.loadURDF('urdf/2link.urdf', useFixedBase = 1)
        self.num_jts = 4 # Joints(including floor and base link which is a fixed one..)
        print('Number of Joints in my arm: ',self.num_jts)
        
        ##here the joints is taken n-1 since base has a fixed joint,hence movable will be n-1
        self.mvbl_jts = list(range(1, self.num_jts - 1))
        print('movable joints:', self.mvbl_jts,' End-effector: 2')
        
    def getJointStates(self):
        joint_states = p.getJointStates(self.rob_id, self.mvbl_jts)
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques
        
    def getJacobian(self, joint_pos):
        eestate = p.getLinkState(self.rob_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eestate
        zero_vec = [0] * len(joint_pos)
        jac_trans, jac_rot = p.calculateJacobian(self.rob_id, self.end_eff_index, com_trn, list(joint_pos), zero_vec, zero_vec)
        j_t = np.asarray(jac_trans)
        j_r = np.asarray(jac_rot)
        j = np.concatenate((j_t, j_r))
        #print('Jacobian:', j)
        
        return j
        
    def solveForwardPositonKinematics(self, joint_pos):
        #print('Forward position kinematics')

        # get end-effector link state
        eeState = p.getLinkState(self.rob_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
        eePose = list(link_trn) + list(p.getEulerFromQuaternion(link_rot))
        #print('End-effector pose:', eePose)
        return eePose
        
    def solveForwardVelocityKinematics(self, joint_pos, joint_vel):
        #print('Forward velocity kinematics')
        J  = self.getJacobian(joint_pos)
        eeVelocity = J @ joint_vel
        #print('End-effector velocity:', eeVelocity)
        return eeVelocity

        
    def calculateDynamicMatrices(self):
        joint_pos, joint_vel, acc = self.getJointStates()
        n_dof = len(self.mvbl_jts)
        InertiaMatrix= np.asarray(p.calculateMassMatrix(self.rob_id, joint_pos))
        GravityMatrix = np.asarray(p.calculateInverseDynamics(self.rob_id, joint_pos, [0.0] * n_dof, [0.0] * n_dof))
        CoriolisMatrix = np.asarray(p.calculateInverseDynamics(self.rob_id, joint_pos, joint_vel, [0.0] * n_dof)) - GravityMatrix
        return InertiaMatrix, GravityMatrix, CoriolisMatrix
        
        
    
            
    
    def imp_con(self):
        p.setRealTimeSimulation(False)
        for i in range(self.num_jts):
            p.changeDynamics(self.rob_id, i, linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
        
        p.setJointMotorControlArray(self.rob_id, self.mvbl_jts,
                                    p.VELOCITY_CONTROL,
                                    forces=[0,0])
        
        # desired velocity is zero
        dxd = [0,0,0,0,0,0]
        
        # stiffness and damping constant
        Kp = 30
        Kd = 15
        # Mm is a desired inertia based matrix which is mostly diagonal
        # Source : http://www.dis.uniroma1.it/~deluca/rob2_en/15_ImpedanceControl.pdf
        temp = np.eye(6)
        Mm = 0.1*temp
    
        
        while True:
            
            xd = np.array([1.34,0,2.95,0.0,1.6,0.0])
            ## practically external force is measured using 6 axis force sensor which measures force applied to end effector also sensorless force control is also possible
            ## F_ext is forces and moments in x y z directions giving a wrench
            F_ext = [-100,0,50,100,0,-50]

          
            jt, djt, _ = self.getJointStates()

           
            x = self.solveForwardPositonKinematics(jt)
            #print("XXXXxx",x)
            x_e = xd - x
            dx = self.solveForwardVelocityKinematics(jt, djt)
            dx_e = dxd - dx
            
            J = self.getJacobian(jt)
            J_inv = np.linalg.pinv(J)
            M_jt, G, Nil = self.calculateDynamicMatrices()
          
            
            Mx = np.dot(np.dot(np.transpose(J_inv), M_jt), J_inv)
            #Mm = Mx
            #Fx = np.dot(np.dot(np.linalg.inv(Mm), Mx),(np.dot(Kp, x_e) + np.dot(Kd, dx_e)))
            #print("1111",Fx)
            #F_w_ext = np.dot((np.dot(np.linalg.inv(Mm), Mx) - np.eye(6)), F_ext)
            #print("2222",F_w_ext)
            #Fx += F_w_ext
            I = np.eye(6)
            f1 = np.dot(np.dot(np.linalg.pinv(Mm), Mx),(np.dot(Kp, x_e) + np.dot(Kd, dx_e)))
            f2 = np.dot((np.dot(np.linalg.inv(Mm), Mx) - I), F_ext)
            ff = 
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
    
   

   
