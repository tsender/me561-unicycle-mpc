#!usr/bin/env python

import math
import numpy as np
import cvxopt

class UnicycleMPC(object):

   def __init__(self, N, T, Q, QN, R, xref, uref, xmin, xmax, umin, umax):
      """Constructor

      Args:
      N: Receding horizon (integer)
      T: Time step (double)
      Q: State cost matrix as numpy array of size (n,n)
      QN: Final state cost matrix as numpy array of size (n,n)
      R: Input cost matrix as numpy array of size (m,m)
      xref: Entire reference trajectory from x0 to xf as numpy array of size (_, 3), where each row is [xref, yref, theta_ref]
      uref: Entire reference input from ur0 to urf as numpy array of size (_, 2), where each row is [vref, ang_vel_ref]
      xmin: Min values of (x - xref) as numpy array of size (1,3)
      xmax: Max values of (x - xref) as numpy array of size (1,3)
      umin: Min values of (u - uref) as numpy array of size (1,2)
      umax: Max values of (u - uref) as numpy array of size (1,2)
      """
      self.n = 3 # 3 states (x, y, theta)
      self.m = 2 # 2 inputs (fwd_vel, ang_vel)
      self.k = 0 # Time step k

      self.N = N
      self.T = T
      self.Q = Q
      self.QN = QN
      self.R = R
      self.xref = xref.reshpe(1,3)
      self.uref = uref.reshpe(1,2)
      self.xmin = xmin.reshpe(1,3)
      self.xmax = xmax.reshpe(1,3)
      self.umin = umin.reshpe(1,2)
      self.umax = umax.reshpe(1,2)

      self.set_QR_cost_matrices()
      
   def set_QR_cost_matrices(self):
      """Setup Qbar and Rbar matrices needed by QP problem"""

      self.Qbar = np.zeros((self.n * self.N, self.n * self.N))
      self.Rbar = np.zeros((self.m * self.N, self.m * self.N))
      
      for i in range(self.N-1):
         j = i * self.n
         k = i * self.m
         self.Qbar[j:j+self.n, j:j+self.n] = self.Q
         self.Rbar[k:k+self.n, k:k+self.n] = self.R

         if i == self.N-2:
            j = j + self.n
            k = k + self.m
            self.Qbar[j:j+self.n, j:j+self.n] = self.QN
            self.Rbar[k:k+self.n, k:k+self.n] = self.R

   def Ak(self, k):
      """Discrete LTV system: x(k+1) = A(k)*x(k) + B(k)*u(k)
         Return A(k)

      Args:
      k: kth instant

      Returns:
      Matrix A(k) of size (n,n)
      """
      A = np.identity(3)
      A[0,2] = -self.uref(k,0) * math.sin(self.xref(k,2)) * self.T
      A[1,2] = self.uref(k,0) * math.cos(self.xref(k,2)) * self.T
      return A

   def Bk(self, k):
      """Discrete LTV system: x(k+1) = A(k)*x(k) + B(k)*u(k)
         Return B(k)

      Args:
      k: kth instant

      Returns:
      Matrix B(k) of size (n,m)
      """
      B = np.zeros((3,2))
      B[0,0] = math.cos(self.xref(k,2)) * self.T
      B[1,0] = math.sin(self.xref(k,2)) * self.T
      B[2,1] = T
      return B

   def Akjl(self, k, j, l):
      """Returns A(k,j,l) = A(k+N-j) * A(k+N-j-1) * A(k+N-j-2) * ... * A(k+l)

      Args:
      k: kth instant
      j: Defines the first element in the multiplication at (k+N-j)
      l: Defines the last element in the multiplication at (k+l)

      Returns:
      Matrix A(k,j,l) of size (n,n)
      """
      Akjl = np.identity(3)
      for i in reversed(range(self.N - j - l + 1)):
         Akjl = Akjl * self.Ak(k + i)
      return Akjl

   def Abar(self, k):
      """Get Abar matrix at kth instant

      Args:
      k: kth instant

      Returns:
      Matrix Abar(k) of size (nN, n)
      """
      Abar = self.Akjl(k, self.N, 0) # A(k)
      for j in reversed(range(1, self.N)): # N-1, N-2, ..., 1
         Abar = np.vstack(Abar, self.Akjl(k,j,0))
      return Abar

   def Bbar(self, k):
      """Get Bbar matrix at kth instant

      Args:
      k: kth instant

      Returns:
      Matrix Bbar(k) of size (nN, mN)
      """
      Bbar = np.zeros((self.n * self.N, self.m * self.N))

      # Fill in the block diagonal elements
      for q in range(self.N):
         i = q * self.n
         j = q * self.m
         Bbar[i:i+self.n, j:j+self.m] = self.Bk(k+q)

      # Now fill in the lower triangular portion
      for r in range(self.N-1): # Across the col groups 0, 1, ,,, N-2
         j = r * self.m
         
         Bkr = self.Bk(k+r)
         for q in range(r+1, self.N): # Down the row groups 1, 2, ..., N-1
            i = q * self.n
            Bbar[i:i+self.n, j:j+self.m] = self.Akjl(k, self.N-q, r+1) * Bkr
      return Bbar

   def update(self, xk):
      """Update model with current xstate, compute control input u(k)

      Solve MPC problem as a QP problem for trajectory tracking
      QP Problem: min 0.5 * u^T * H * u + f^T * u^T
                  subject to: G * u <= w

                  where u = u(k) - uref(k)
      
      Args:
      x: current state x(k) as numpy array of size (1,3), do NOT subtract xref(k)

      Returns:
      (Full) control input u(k) = uerr + uref(k) to apply to the system
      """
      xerr = (xk.reshape(1,3) - self.xref[k,:]).reshape(3,1)
      Abar = self.Abar(self.k)
      Bbar = self.Bbar(self.k)

      # QP matrices
      # Recall, np.dot is normal matrix multiplication
      Hk = 2 * Bbar.T.dot(self.Qbar).dot(BBar) + self.Rbar # Size (mN, mN)
      fk = 2 * Bbar.T.dot(self.Qbar).dot(Abar).dot(xerr) # Size (mN, 1)

      # G matrix
      ImN = np.identity(self.m * self.N)
      Gk = np.concatenate((ImN, -ImN, Bbar, -Bbar)) # Size (2mN + 2nN, mN)

      # W Vector
      w1 = (self.umax - self.uref[self.k:self.k + self.N, :]).flatten().reshape(-1,1) # Flatten into 2D col vector
      w2 = (-self.umin + self.uref[self.k:self.k + self.N, :]).flatten().reshape(-1,1)
      w3 = (self.xmax - self.xref[self.k:self.k + self.N, :]).flatten().reshape(-1,1) - Abar.dot(xerr)
      w4 = (-self.xmin + self.xref[self.k:self.k + self.N, :]).flatten().reshape(-1,1) + Abar.dot(xerr)
      wk = np.concatenate([w1, w2, w3, w4]) # Size (2mN + 2nN, 1)

      # Setup and solve QP problem
      Hk = cvxopt.matrix(Hk)
      fk = cvxopt.matrix(fk)
      Gk = cvxopt.matrix(Gk)
      wk = cvxopt.matrix(wk)

      cvxopt.solvers.options['show_progress'] = False
      usol = cvxopt.solvers.qp(Hk, fk, Gk, wk)
      self.k = self.k + 1
      return usol[0:self.m] + self.uref[self.k-1, :].reshape(3,1)

      # TODO: verify cvxopt solved problem
      # TODO: Handle case when xref and uref are near the end of the time-series (have to pad arrays with last elements)


# Test Code
# Akjl = np.identity(3)
# A2 = 2 * Akjl
# A2[0,1] = 5
# vec3 = np.array([[1.], [2.], [3.]])
# ved4 = vec3.flatten()
# print vec3
# print A2
# print A2.T.dot(A2.T)
# print A2.dot(vec3)
# print np.concatenate((vec3, vec3), axis=1)
# print vec3 - A2 # adds to every row
# print A2.flatten().reshape(-1,1)

# Q = 2*cvxopt.matrix([ [2, .5], [.5, 1] ])
# p = cvxopt.matrix([1.0, 1.0])
# G = cvxopt.matrix([[-1.0,0.0],[0.0,-1.0]])
# h = cvxopt.matrix([0.0,0.0])
# A = cvxopt.matrix([1.0, 1.0], (1,2))
# b = cvxopt.matrix(1.0)
# cvxopt.solvers.options['show_progress'] = False
# sol=cvxopt.solvers.qp(Q, p, G, h, A, b)
# print(sol['x'])

# print vec3.shape
# col1 = np.array([[1], [1], [1]])
# col2 = 2 * col1
# col12 = np.hstack((col1, col2))
# print col12
# Akjl[:,0:0+2] = col12
# print Akjl

# N = 4
# for i in reversed(range(1,N)):
#    print i
