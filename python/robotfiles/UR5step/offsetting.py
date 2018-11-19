Toffset = eye(4)
Toffset[:3,:3] = rotationMatrixFromAxisAngle([0,-pi/2,0])

offsetTranslations = [
  [0.003,0,0.0], # link0
  [-0.02215,0.0,0.0], # link1
  [-0.0862,0.06779,0.0], # link2
  [-0.5112,0.0663,0.0], # link3
  [-0.90345,0.05663,0.0], # link4
  [-0.94493,0.110,0.0], # link5
  [-0.9982,0.15138,0.0]  # link6
]

# J1: 86.2      75.3    0    axis 0.44 -0.44 -0.78   103.978
# J2: 511.2     75.3    0    axis 0.16 0.16 -0.97    91.5871 deg
# J3: 903.45    63.75   0    axis -0.27 -0.27 0.92   94.6699 deg
# J4: 951.95   110.0   0    axis 0.0  0.83 -0.56    180.0 deg
# J5: 998.2    158.5   0    axis 0.64 0.64 -0.43    226.297 deg

for linkindex in range(7):
  filename = 'link%d_simpl.stl'%linkindex
  body0 = env.ReadKinBodyURI(filename, {'scalegeometry': '0.001 0.001 0.001'})
  Toffset[:3,3] = offsetTranslations[linkindex] 
  T = linalg.inv(Toffset)
  trimesh = body0.GetLinks()[0].GetCollisionData()

  for i in range(len(trimesh.vertices)):
    pos = [0, 0, 0, 1]
    pos[:3] = trimesh.vertices[i]
    trimesh.vertices[i] = dot(T, pos)[:3]

  body_new = RaveCreateKinBody(env, '')
  body_new.InitFromTrimesh(trimesh, True)
  
  body_new.SetName(body0.GetName())
  geom = body_new.GetLinks()[0].GetGeometries()[0]
  geom.SetDiffuseColor(body0.GetLinks()[0].GetGeometries()[0].GetDiffuseColor())
  geom.SetAmbientColor([0.0,0.0,0.0])
  
  env.Remove(body0)
  env.Add(body_new)
  env.Save('%s_rot.mujin.dae'%body0.GetName())
  env.Remove(body_new)
