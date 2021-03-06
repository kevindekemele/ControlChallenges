# Sage script to optimize the double pendulum controller

var('x0 dx0 ddx0                               m0   ')
var('x1 dx1 ddx1 y1 dy1 ddy1 th1 dth1 ddth1 T1 m1 L1')
var('x2 dx2 ddx2 y2 dy2 ddy2 th2 dth2 ddth2 T2 m2 L2')
var('g')
var('c1 c2 c3 c4 c5 c6')

equations = [
	# kinetics
	m0*ddx0==T1*sin(th1) + c1*x0 + c2*dx0 + c3*th1 + c4*dth1 + c5*th2 + c6*dth2,
	m1*ddx1==-T1*sin(th1)+T2*sin(th2),
	m2*ddx2==-T2*sin(th2),
	m1*ddy1==-m1*g-T1*cos(th1)+T2*cos(th2),	
	m2*ddy2==-m2*g-T2*cos(th2),

	# kinematics
	-L1*(ddth1*cos(th1)-sin(th1)*dth1**2) == -ddx1+ddx0,
	-L2*(ddth2*cos(th2)-sin(th2)*dth2**2) == -ddx2+ddx1,

	L1*(cos(th1)*dth1**2+sin(th1)*ddth1) == -ddy1,
	L2*(cos(th2)*dth2**2+sin(th2)*ddth2) == -ddy2 + ddy1,
]

# Rewrite equations as a function F(X,Y)=0
EqF = [equations[i].lhs()-equations[i].rhs() for i in range(9)]
X = [x0,dx0,th1,dth1,th2,dth2] # inputs, i.e. system state
Y = [ddx0,ddth1,ddth2,ddx1,ddx2,ddy1,ddy2,T1,T2] # state derivatives

# Values for the parameters
vals = {m0: 10,m1: 2,m2: 4,L1: 0.618,L2: 1,g: 2}


# Balance point (upright pendulum)
X0 = {x:0 for x in X}

# Find Y values in the balance point
Y0_eq = solve([e.lhs().subs(vals).subs(X0) == e.rhs().subs(vals).subs(X0) for e in equations],Y)[0]
Y0 = {e.lhs():e.rhs() for e in Y0_eq}


# Find Jacobi matrices of F in (X0,Y0)
dFdY = matrix(SR,9,9)
dFdX = matrix(SR,9,6)
for i in range(9):
	for j in range(9):
		dFdY[i,j] = EqF[i].derivative(Y[j]).subs(X0).subs(Y0)
for i in range(9):
	for j in range(6):
		dFdX[i,j] = EqF[i].derivative(X[j]).subs(X0).subs(Y0)

# Implicit function theorem gives the derivatives of Y with respect to X
# Only the first three lines needed, they correspond to [ddx0,ddth1,ddth2]
DDX = -dFdY.solve_right(dFdX).subs(vals)[0:3,:]

# Linearized state space model around the balance point
SS = matrix(SR,6,6)
SS[0,1] = 1
SS[1,:] = DDX[0,:]
SS[2,3] = 1
SS[3,:] = DDX[1,:]
SS[4,5] = 1
SS[5,:] = DDX[2,:]

# Put in numbers for the controller gains
SS_num = lambda p: SS.subs({c1:p[0],c2:p[1],c3:p[2],c4:p[3],c5:p[4],c6:p[5]})

import numpy as np

# Largest eigenvalue of the linearized system. It needs to be negative for the system to be stable.
eigs = lambda x: np.max(np.real(np.linalg.eig(np.array(SS_num(x),dtype=np.float64))[0]))


# Poor man's optimization
# Throw random numbers at the matrix and select those with smaller eigenvalue.

# solution from a previous run
min_x = [-0.2746958872107124, -2.37483873236478, 300.8150048652969, 92.69138090493473, -286.03604596562946, -127.69638629632554]
min_eig = eigs(min_x)
print min_eig,min_x

i=0
while True:
	if(i%4==0):
		new_x = 10*np.random.randn(6)
	elif(i%4==1):
		new_x = min_x + np.random.randn(6)
	elif(i%4==2):
		new_x = min_x + .1*np.random.randn(6)
	else:
		new_x = min_x + 0.01*np.multiply(np.random.randn(6),min_x)	

	new_eig = eigs(new_x)
	if(new_eig < min_eig):
		min_eig = new_eig
		min_x = new_x
		print min_eig,min_x.tolist()
	i+=1