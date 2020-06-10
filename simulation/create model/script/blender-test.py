#!/usr/bin/env python3
import numpy as np
import bpy

class MakeJointModel:
    def __init__(self):
        #make material
        bpy.data.materials.new(name = 'link')
        mat = bpy.data.materials['link']
        mat.use_nodes = False
        mat.diffuse_color = (1,1,1,1) 
        bpy.data.materials.new(name = 'joint')
        mat = bpy.data.materials['joint']
        mat.use_nodes = False
        mat.diffuse_color = (0,0.5,0,1) 

    def make_link(self, name, size, length, pos, att):
        pos_ = np.array( [[0], [0], [ 0.5*length]] )
        att_ = np.array( [[0], [0], [0]] )
        bpy.ops.mesh.primitive_cylinder_add(depth = length, radius=size, location=pos_, rotation=att_)
        #
        bpy.ops.transform.rotate(value=att[1], orient_axis='X', orient_type='GLOBAL')
        bpy.ops.transform.rotate(value=att[1], orient_axis='Y', orient_type='GLOBAL')
        bpy.ops.transform.rotate(value=att[2], orient_axis='Z', orient_type='GLOBAL')       
        bpy.ops.transform.translate(value=pos, constraint_axis=( True, True, True)) 
        # change color
        mat = bpy.data.materials['link']
        bpy.ops.object.material_slot_add()
        bpy.context.object.active_material=mat
        # change name
        obj = bpy.context.object
        obj.name = name
        obj.data.name = name
    
    def make_base_link(self, name, size, pos, att):
        depth_ = 2*size
        pos_ = np.array( [[0], [0], [ 0.5*depth_]] )
        att_ = np.array( [[0], [0], [0]] )
        bpy.ops.mesh.primitive_cone_add(depth=depth_, radius1=4*size*np.sqrt(2), radius2=4*size,  location=pos_, rotation=att_)
        #
        bpy.ops.transform.rotate(value=att[1], orient_axis='X', orient_type='GLOBAL')
        bpy.ops.transform.rotate(value=att[1], orient_axis='Y', orient_type='GLOBAL')
        bpy.ops.transform.rotate(value=att[2], orient_axis='Z', orient_type='GLOBAL')   
        bpy.ops.transform.translate(value=pos, constraint_axis=( True, True, True))  
        # change color
        mat = bpy.data.materials['link']
        bpy.ops.object.material_slot_add()
        bpy.context.object.active_material=mat  
        # change name
        obj = bpy.context.object
        obj.name = name
        obj.data.name = name

    def make_rotational_joint(self, name, size, pos, att):
        depth_ = size
        objects = []
        holes = []
        pos1 = np.array( [[0], [0], [ 0.5*depth_]] )
        pos2 = np.array( [[0], [0], [-0.5*depth_]] )
        direct1 = np.array( [[0], [0], [0]] )
        direct2 = np.array( [[np.pi], [0], [0]] )
        
        #for hole object
        bpy.ops.mesh.primitive_cylinder_add(depth = 0.5*depth_, radius = 0.5*depth_, location=2*pos1, rotation=direct1)
        holes.append(bpy.context.object)
        bpy.ops.mesh.primitive_cylinder_add(depth = 0.5*depth_, radius = 0.5*depth_, location=2*pos2, rotation=direct2)
        holes.append(bpy.context.object)
        
        #make cone1
        bpy.ops.mesh.primitive_cone_add(depth=depth_, radius1=size*np.sqrt(2), radius2=size,  location=pos1, rotation=direct1)
        objects.append(bpy.context.object)
        #make hole of cone1
        bpy.ops.object.modifier_add(type='BOOLEAN')
        bpy.context.object.modifiers["Boolean"].object = holes[0]
        bpy.context.object.modifiers["Boolean"].operation = 'DIFFERENCE'
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Boolean")

        #make cone2
        bpy.ops.mesh.primitive_cone_add(depth=depth_, radius1=size*np.sqrt(2), radius2=size,  location=pos2, rotation=direct2)
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
        # join both of cone
        for obj in objects:
            obj.select_set(True)
        bpy.ops.object.join()
        #
        cursor = bpy.context.scene.cursor
        cursor.location = (0,0,0)
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='MEDIAN')
        bpy.ops.transform.rotate(value=att[0], orient_axis='X', orient_type='GLOBAL')
        bpy.ops.transform.rotate(value=att[1], orient_axis='Y', orient_type='GLOBAL')
        bpy.ops.transform.rotate(value=att[2], orient_axis='Z', orient_type='GLOBAL')           
        bpy.ops.transform.translate(value=pos, constraint_axis=( True, True, True))
        # change color
        mat = bpy.data.materials['joint']
        bpy.ops.object.material_slot_add()
        bpy.context.object.active_material=mat
        # change name
        obj.name = name
        obj.data.name = name
        
def tra( x, y, z):
    return np.array([[x], [y], [z]])
    
def rot(roll, pitch, yaw):
    return np.array([[roll], [pitch], [yaw]])

class JointRobot:
    def __init__(self):
        self.scale = 1.0
        self.setting = [ \
           ['J', 'Yaw'], ['L', 3.0], \
           ['J', 'Rol'], ['L', 3.0], \
           ['J', 'Pit'], ['L', 3.0], \
           ['J', 'Yaw'], ['L', 3.0], \
           ['J', 'Rol'], ['L', 3.0], \
           ['J', 'Pit'], ['L', 3.0], \
           ['J', 'Yaw'], ['L', 3.0], \
           ['J', 'Rol'], ['L', 3.0], \
           ['J', 'Pit'], ['L', 3.0], \
           ['J', 'Yaw'], ['L', 3.0]]

    def create(self, init_pos, init_att):
        creater = MakeJointModel()
        pos = np.array(init_pos)
        att = np.array(init_att)
        joint_num = 1
        link_num = 1
        creater.make_base_link("BaseLink", self.scale, pos, att)
        pos += tra(0.0, 0.0, 3.0*self.scale)
        for parts in self.setting:
            if parts[0] == 'J':
                if parts[1] == 'Rol':
                    att = rot(0.0, np.pi/2, 0.0)
                elif parts[1] == 'Pit':
                    att = rot(np.pi/2, 0.0, 0.0)
                elif parts[1] == 'Yaw':
                    att = rot(0.0, 0.0, 0.0)
                creater.make_rotational_joint("Joint"+str(joint_num), self.scale, pos, att )
                pos += tra(0.0, 0.0, self.scale)
                joint_num += 1
            elif parts[0] == 'L':
                att = init_att
                creater.make_link("Link"+str(link_num), self.scale, float(parts[1]), pos, att)
                pos += tra(0.0, 0.0, float(parts[1])+self.scale)
                link_num += 1
        
        bone = []
        bone_num = 0
        bpy.ops.object.add(type='ARMATURE', enter_editmode=True, align='WORLD', location=init_pos)
        amt = bpy.context.object
        amt.name = 'RobotArmature'

        bpy.ops.object.mode_set(mode='EDIT')
        b = amt.data.edit_bones.new('BaseLinkBone')
        bone.append(b)
        bone[bone_num].head = np.array(init_pos)
        end_pos = tra(0.0, 0.0, 2.0*self.scale)
        bone[bone_num].tail = end_pos
        bone[bone_num].use_deform = False
        bone_num += 1
        for bones in self.setting:
            if bones[0] == 'J':
                end_pos += tra(0.0, 0.0, self.scale)
                bone[bone_num-1].tail = end_pos
            elif bones[0] == 'L':
                b = amt.data.edit_bones.new('LinkBone'+str(bone_num))
                bone.append(b)
                bone[bone_num].head =  bone[bone_num-1].tail
                end_pos += tra(0,0,float(bones[1]) + self.scale)
                bone[bone_num].tail = end_pos
                bone[bone_num].parent =  bone[bone_num-1]
                bone[bone_num].use_deform = False
                bone_num += 1
        bpy.context.object.show_in_front = True   

if __name__ == '__main__':  
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(True)
    
    robot = JointRobot()
    robot.create(tra(0.0, 0.0, 0.0), rot(0.0, 0.0, 0.0))