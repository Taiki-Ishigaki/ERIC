#!/usr/bin/env python3
import numpy as np
import bpy

class MakeJointModel:
#    def __init__(self):

    def make_link(self, name, size, length, pos, att):
        bpy.ops.mesh.primitive_cylinder_add(depth = length, radius=size, location=pos, rotation=att)
        obj = bpy.context.object
        obj.name = name
        obj.data.name = name

    def make_rotational_joint(self, name, size, pos, att):
        depth_ = size
        objects = []
        holes = []
        pos1 = np.array( [[0], [0], [ depth_/2]] )
        pos2 = np.array( [[0], [0], [-depth_/2]] )
        direct1 = np.array( [[0], [0], [0]] )
        direct2 = np.array( [[np.pi], [0], [0]] )
        
        #for hole object
        bpy.ops.mesh.primitive_cylinder_add(depth = depth_/2, radius=depth_/2, location=pos+pos1*2, rotation=att+direct1)
        holes.append(bpy.context.object)
        bpy.ops.mesh.primitive_cylinder_add(depth = depth_/2, radius=depth_/2, location=pos+pos2*2, rotation=att+direct2)
        holes.append(bpy.context.object)
        
        #make cone1
        bpy.ops.mesh.primitive_cone_add(depth=depth_, radius1=size*np.sqrt(2), radius2=size,  location=pos+pos1, rotation=att+direct1)
        objects.append(bpy.context.object)
        #make hole of cone1
        bpy.ops.object.modifier_add(type='BOOLEAN')
        bpy.context.object.modifiers["Boolean"].object = holes[0]
        bpy.context.object.modifiers["Boolean"].operation = 'DIFFERENCE'
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Boolean")

        #make come2
        bpy.ops.mesh.primitive_cone_add(depth=depth_, radius1=size*np.sqrt(2), radius2=size,  location=pos+pos2, rotation=att+direct2)
        objects.append(bpy.context.object)
        #make hole
        bpy.ops.object.modifier_add(type='BOOLEAN')
        bpy.context.object.modifiers["Boolean"].object = holes[1]
        bpy.context.object.modifiers["Boolean"].operation = 'DIFFERENCE'
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Boolean")
        
        bpy.ops.object.select_all(action="DESELECT") 
        
        # delete holes
        for obj in holes:
            obj.select_set(True)
        bpy.ops.object.delete()
        
        for obj in objects:
            obj.select_set(True)
        
        # join both of cone
        bpy.ops.object.join()
        obj.name = name
        obj.data.name = name
        
class Kinematics:
#    def __init__
    
    def vertical(x, y, z):
        return np.array([[x], [y], [z]])

if __name__ == '__main__':
    robot_scale = 1.0
    
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(True)
    
    creater = MakeJointModel()
    creater.make_rotational_joint("joint1", robot_scale, np.array([[0], [0], [10]]), np.array([[0], [0], [0]]))
    creater.make_rotational_joint("joint2", robot_scale, np.array([[0], [5], [0]]), np.array([[0], [0], [0]]))
    creater.make_rotational_joint("joint3", robot_scale, np.array([[5], [0], [0]]), np.array([[0], [0], [0]]))
    
    creater.make_link("link1", robot_scale, 4.0, np.array([[0], [0], [0]]), np.array([[0], [0], [0]]))