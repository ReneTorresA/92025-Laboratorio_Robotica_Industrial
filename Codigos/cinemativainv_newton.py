import numpy as np
import matplotlib.pyplot as plt



posobj = np.array([[0.7],[1.4]])

#geometria
l1 = 1
l2 = 1
theta1 = np.pi/4
theta2 = np.pi/1

#Forward Kinematics
def forward_kinematics(theta1,theta2,l1,l2):
    x = l1*np.cos(theta1) + l2*np.cos(theta1+theta2)
    y = l1*np.sin(theta1) + l2*np.sin(theta1+theta2)
    pos = np.array([[x],[y]])
    return pos

#Jacobian
def Jacobian(theta1,theta2,l1,l2):
    J11 = -l1*np.sin(theta1) - l2*np.sin(theta1+theta2)
    J12 = -l2*np.sin(theta1+theta2)
    J21 = l1*np.cos(theta1) + l2*np.cos(theta1+theta2)
    J22 = l2*np.cos(theta1+theta2)
    J = np.array([[J11,J12],[J21,J22]])
    return J



for i in range(0,1000):
    pos = forward_kinematics(theta1,theta2,l1,l2)
    J = Jacobian(theta1,theta2,l1,l2)
    #---------------------------------------------------------    
    #pseudo_inv = np.linalg.pinv(J) #Inversa de Moore Penrose
    #---------------------------------------------------------

    #---------------------------------------------------------
    #Pseudoinversa método manual
    pseudopartial = np.linalg.inv(np.dot(J,np.transpose(J)))
    pseudo_inv = np.dot(np.transpose(J),pseudopartial)
    #---------------------------------------------------------
    


    theta = np.array([[theta1],[theta2]])
    theta = theta + np.dot(pseudo_inv,(posobj-pos))
    theta1 = theta[0,0]
    theta2 = theta[1,0]
    if np.linalg.norm(posobj-pos) < 0.01:
        print("Numero de iteraciones: ",i)
        print("Error: ",np.linalg.norm(posobj-pos))
        print("Posición final: ",pos, "[x,y]")
        print("Ángulos finales: [",theta1,",",theta2, "] [rad]")
        break


#Plot posición del brazo robótico
plt.plot([0,l1*np.cos(theta1),l1*np.cos(theta1)+l2*np.cos(theta1+theta2)],[0,l1*np.sin(theta1),l1*np.sin(theta1)+l2*np.sin(theta1+theta2)], 'b-')

#plt.plot(posobj[0,0],posobj[1,0],'ro')
plt.plot(posobj[0,0],posobj[1,0], 'go', markersize=10, label='Posición objetivo')
plt.axis('equal')
plt.xlim(-2.5, 2.5)
plt.ylim(-0.5, 4)
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.title('Brazo Robótico')

plt.show()