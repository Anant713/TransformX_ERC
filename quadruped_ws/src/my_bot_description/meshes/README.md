# meshes/

Place the STL files exported from SolidWorks here.
The URDF references these files by name, e.g.:
  package://my_bot_description/meshes/base_link.STL
  package://my_bot_description/meshes/shoulder 1.STL
  ...

Full list expected by my_robot.urdf:
  base_link.STL
  shoulder 1.STL   femur 1.STL   leg 1.STL   wheel 1.STL
  shoulder 2.STL   femur 2.STL   leg 2.STL   wheel 2.STL
  shoulder 3.STL   femur 3.STL   leg 3.STL   wheel 3.STL
  shoulder 4.STL   femur 4.STL   leg 4.STL   wheel 4.STL

Note: Filenames contain spaces exactly as exported by SW2URDF.
