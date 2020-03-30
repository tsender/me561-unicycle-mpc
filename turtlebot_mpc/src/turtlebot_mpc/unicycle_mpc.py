#!usr/bin/env python

import math
import numpy as np
import cvxopt

class UnicycleMPC(object):

   def __init__(self, xref, uref, N, T):
      """Constructor

      Args:
      xref: Entire reference trajectory from x0 to xf as numpy array of size (_, 3), where each row is [xref, yref, theta_ref]
      uref: Entire reference input from ur0 to urf as numpy array of size (_, 2), where each row is [vref, ang_vel_ref]
      N: Receding horizon (integer)
      T: time step (double)
      """
      self.xref = xref
      self.uref = uref
      self.N = N
      self.n = 3 # 3 states (x, y, theta)
      self.m = 2 # 2 inputs (fwd_vel, ang_vel)
      self.T = T

   def Ak(self, k):
      """Discrete LTV system: x(k+1) = A(k)*x(k) + B(k)*u(k)
         Return A(k)

      Args:
      k: kth instant

      Returns:
      Matrix A(k) of size (n,n)
      """

      A = np.identity(3)
      A[0,2] = -self.uref(k,0) * math.sin(self.uref(k,1)) * self.T
      A[1,2] = self.uref(k,0) * math.cos(self.uref(k,1)) * self.T
      return A

   def Bk(self, k):
      """Discrete LTV system: x(k+1) = A(k)*x(k) + B(k)*u(k)
         Return B(k)

      Args:
      k: kth instant

      Returns:
      Matrix B(k) of size (n,m)
      """

      B = np.zeros((2,3))
      B[0,0] = math.cos(self.uref(k,1)) * self.T
      B[1,0] = math.sin(self.uref(k,1)) * self.T
      B[2,1] = 1
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
      # Abar = np.zeros((self.n * self.N, self.n))
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
         
         for q in range(r+1, self.N): # Down the row groups 1, 2, ..., N-1
            i = q * self.n
            Bbar[i:i+self.n, j:j+self.m] = self.Akjl(k, self.N-q, r+1) * self.Bk(k+r)
      return Bbar

# Akjl = np.identity(3)
# print Akjl
# col1 = np.array([[1], [1], [1]])
# col2 = 2 * col1
# col12 = np.hstack((col1, col2))
# print col12
# Akjl[:,0:0+2] = col12
# print Akjl

# N = 4
# for i in reversed(range(1,N)):
#    print i
