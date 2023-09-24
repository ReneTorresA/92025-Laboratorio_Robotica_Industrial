import numpy as np

cos=np.cos; sin=np.sin
xd = np.array([1.2, 1.2]) # Desired value in the Cartesian space 
q  = np.array([0.5, 0.5]) # Initial value in the joint space
epsilon = 1e-3
max_iter = 1000    # Maximum number of iterations
alpha = 0.3
# Iterations: Gradient descent
for i in range(max_iter):
    q1 = q[0]
    q2 = q[1]
    J = np.array([[-sin(q1)-sin(q1+q2), -sin(q1+q2)],
                  [ cos(q1)+cos(q1+q2),  cos(q1+q2)]])
    f = np.array([cos(q1)+cos(q1+q2), sin(q1)+sin(q1+q2)])
    e = xd-f
    q = q + alpha*np.dot(J.T, e)
    # End condition
    if (np.linalg.norm(e) < epsilon):
        break
print('Ángulos:')
print(q)

print('Comprobación  (f(q)):')
print([cos(q[0])+cos(q[0]+q[1]), sin(q[0])+sin(q[0]+q[1])])
