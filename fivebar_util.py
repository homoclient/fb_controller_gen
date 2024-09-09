import numpy as np

def _is_reachable(x, y, A, B, C):
  far = (A+B)**2
  close = 0 if np.isclose(A-B, 0) else (A-B)**2
  d1s = (x+C/2)**2 + y**2
  d2s = (x-C/2)**2 + y**2
  return (np.less_equal(close, d1s) & np.less_equal(d1s, far)) & (np.less_equal(close, d2s) & np.less_equal(d2s, far))

def _is_valid(theta1, theta2, A, B, C):
  r1r2s = (C+B*(np.cos(theta2)-np.cos(theta1)))**2 + (B*(np.sin(theta2)-np.sin(theta1)))**2
  return r1r2s <= 4*A**2

def _tell_mode(theta1, theta2, x ,y, B, C):
  working_mode = [np.linalg.det([[np.cos(theta1), np.sin(theta1)], [x+C/2-B*np.cos(theta1), y-B*np.sin(theta1)]]) > 0,
                  np.linalg.det([[np.cos(theta2), np.sin(theta2)], [x-C/2-B*np.cos(theta2), y-B*np.sin(theta2)]]) > 0]
  r1 = B*np.array([np.cos(theta1), np.sin(theta1)])+np.array([-C/2, 0.0])
  r2 = B*np.array([np.cos(theta2), np.sin(theta2)])+np.array([C/2, 0.0])
  r1p = np.array([x,y])-r1
  r2p = np.array([x,y])-r2
  assembly_mode = int(np.linalg.det([r1p, r2p]) > 0)
  return (assembly_mode, working_mode)

def _fk(theta1, theta2, assembly_mode, A, B, C):
  D = (B**2/2)*(1-np.cos(theta1-theta2))+(B*C/2)*(np.cos(theta2)-np.cos(theta1))+C**2/4
  if np.isclose(D, 0): # infinitly many solutions
    return np.array([np.nan, np.nan])
  tmp = np.sqrt(A**2/D - 1)
  assy = [-1, 1][assembly_mode > 0]
  p = np.array([np.cos((theta1+theta2)/2), np.sin((theta1+theta2)/2)])*B*(
    np.cos((theta1-theta2)/2)+assy*tmp*np.sin((theta1-theta2)/2)
  ) + assy*np.array([0, tmp*C/2])
  return p

def _ik(x, y, working_mode, A, B, C):
  d1s = (x+C/2)**2 + y**2
  d2s = (x-C/2)**2 + y**2
  wm = [[-1, 1][working_mode[0] > 0], [-1, 1][working_mode[1] > 0]]
  m1 = np.array([[x + C/2, y], [y, -x - C/2]])
  m2 = np.array([[x - C/2, y], [y, -x + C/2]])
  v1 = np.array([B**2 - A**2 + d1s, wm[0]*np.sqrt(4*B**2*d1s - (B**2 - A**2 + d1s)**2)])
  v2 = np.array([B**2 - A**2 + d2s, wm[1]*np.sqrt(4*B**2*d2s - (B**2 - A**2 + d2s)**2)])
  r1 = m1@v1
  r2 = m2@v2
  return np.array([np.arctan2(r1[1], r1[0]), np.arctan2(r2[1], r2[0])])

def _jacobian(theta1, theta2, x, y, assembly_mode, A, B, C):
  m2 = np.diag([(x+C/2)*np.sin(theta1)-y*np.cos(theta1), -(x-C/2)*np.sin(theta2)+y*np.cos(theta2)])
  if np.isclose(m2[0,0], 0) or np.isclose(m2[1,1], 0):
    return np.array([[np.nan, np.nan], [np.nan, np.nan]])
  m1 = np.array([[B*np.sin(theta2)-y, B*np.sin(theta1)-y], [-B*np.cos(theta2)+x-C/2, -B*np.cos(theta1)+x+C/2]])
  assy = [-1, 1][assembly_mode > 0]
  den = B*np.sin(theta2-theta1)+x*(np.sin(theta1)-np.sin(theta2))+y*(np.cos(theta2)-np.cos(theta1)+C/B)-C/2*(np.sin(theta1)+np.sin(theta2))
  if np.isclose(den, 0):
    return np.array([[np.inf, np.inf], [np.inf, np.inf]])
  J = assy * m1@m2/den
  return J

def _load_coriolis_accel(x, y, vx, vy, working_mode, A, B, C):
  d1s = (x+C/2)**2 + y**2
  d2s = (x-C/2)**2 + y**2
  d1 = np.sqrt(d1s)
  d2 = np.sqrt(d2s)
  dd1s = (vx*(x+C/2)+vy*y)**2/d1s
  dd2s = (vx*(x-C/2)+vy*y)**2/d2s
  ddd1 = (vx**2-vy**2-dd1s)/d1
  ddd2 = (vx**2-vy**2-dd2s)/d2
  ddphi = np.r_[2*(vx*y-vy*(x+C/2))*(vx*(x+C/2)+vy*y)/d1s**2, 2*(vx*y-vy*(x-C/2))*(vx*(x-C/2)+vy*y)/d2s**2]
  dpsi1dd1 = (A**2-B**2+d1s)/(2*B*d1s*np.sqrt(1-(-A**2+B**2+d1s)**2/(4*B**2*d1s)))
  dpsi2dd2 = (A**2-B**2+d2s)/(2*B*d2s*np.sqrt(1-(-A**2+B**2+d2s)**2/(4*B**2*d2s)))
  ddpsi1ddd1 = (A**6-(B**2-d1s)**3-A**4*(3*B**2+5*d1s)+A**2*(3*B**4+2*B**2*d1s+3*d1s**2))/(d1s*(2*A**2*(B**2+d1s)-A**4-(B**2-d1s)**2)**1.5)
  ddpsi2ddd2 = (A**6-(B**2-d2s)**3-A**4*(3*B**2+5*d2s)+A**2*(3*B**4+2*B**2*d2s+3*d2s**2))/(d2s*(2*A**2*(B**2+d2s)-A**4-(B**2-d2s)**2)**1.5)
  wm = np.r_[[-1, 1][working_mode[0] > 0], [-1, 1][working_mode[1] > 0]]
  ddpsi = wm*np.r_[ddpsi1ddd1*dd1s+dpsi1dd1*ddd1,ddpsi2ddd2*dd2s+dpsi2dd2*ddd2]
  return ddphi+ddpsi

class Fivebar():
  def __init__(self, A: float, B: float, C: float):
    self.A, self.B, self.C = A, B, C
    self.workspaceAABB_x = 2*(A+B)-C
    self.workspaceAABB_y = 2*np.sqrt((A+B)**2 - C**2/4)
  
  def is_reachable(self, x: float, y: float):
    return _is_reachable(x, y, self.A, self.B, self.C)
  
  def is_valid(self, theta1: float, theta2: float):
    return _is_valid(theta1, theta2, self.A, self.B, self.C)
  
  def tell_mode(self, theta1: float, theta2: float, x: float, y: float):
    return _tell_mode(theta1, theta2, x, y, self.B, self.C)
  
  def fk(self, theta1: float, theta2: float, assembly_mode):
    return _fk(theta1, theta2, assembly_mode, self.A, self.B, self.C)
  
  def ik(self, x: float, y: float, working_mode):
    return _ik(x, y, working_mode, self.A, self.B, self.C)
  
  def jacobian(self, theta1: float, theta2: float, x: float, y: float, assembly_mode):
    return _jacobian(theta1, theta2, x, y, assembly_mode, self.A, self.B, self.C)
  
  def load_coriolis_accel(self, x: float, y: float, vx: float, vy: float, working_mode):
    return _load_coriolis_accel(x, y, vx, vy, working_mode, self.A, self.B, self.C)
  
  def workspace_plot_gen(self, res_x: int, res_y: int):
    y, x = np.mgrid[-self.workspaceAABB_y/2:self.workspaceAABB_y/2:complex(0,res_y), -self.workspaceAABB_x/2:self.workspaceAABB_x/2:complex(0,res_x)]
    return self.is_reachable(x, y)