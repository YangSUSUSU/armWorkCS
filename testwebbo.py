# from urdf2webots.importer import convertUrdfFile
# import urdf2webots
# print(urdf2webots.__file__)

# urdf_path = '/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model/humanoid.urdf'
# output_path = '/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model/humanoid.proto'
# convertUrdfFile(input = urdf_path,output=output_path)



from urdf2webots.importer import convertUrdfFile
convertUrdfFile(input = '/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model/humanoid.urdf')