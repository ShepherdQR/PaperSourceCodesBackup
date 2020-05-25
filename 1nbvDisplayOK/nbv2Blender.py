'''
@Author: Shepherd Qirong
@Date: 2020-04-27 13:28:50
@Github: https://github.com/ShepherdQR
@LastEditors: Shepherd Qirong
@LastEditTime: 2020-04-28 20:19:08
@Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
'''

"""
    x y z , rz ry rx;
"""


import numpy as np
import time, bpy
timeCur = time.strftime("%Y%m%d%H%M%S", time.localtime())
time_4=int(timeCur[-4:])





fileNameIn = "/home/jellyfish/cpps/FinalPaperSourceCodes/nbvDisplay/planViews20200427132449.txt"



def basicAdd( part ="Cube" , name = "DefaultN", location =(0, 0, 0), rotation = (0, 0, 0), scale =(0.1, 0.1, 0.1)  ):    
    if( part == "Cube"  ):
        bpy.ops.mesh.primitive_cube_add()
    if ( part == "Cone"  ):
        bpy.ops.mesh.primitive_cone_add()
    obj_j = bpy.data.objects[part]
    obj_j.location = location
    obj_j.rotation_euler = rotation
    obj_j.scale = scale
    if(name != "DefaultN"):
        obj_j.name = name



def lockfunction( part = "Default", lock =True, location = 1, rotation = 1, scale = 1):
    if( part == "Default" ):
        print("default")

    if(location==1):
        for i in range(3):
            bpy.data.objects[part].lock_location[i]= lock
    if(rotation==1):
        for i in range(3):
            bpy.data.objects[part].lock_rotation[i]= lock
    if(scale==1):
        for i in range(3):
            bpy.data.objects[part].lock_scale[i]= lock


def parentObjects(childName, parentName):
    ## I used about three or four hours to figure it out, then the thing is done. This is the key point.
    #bpy.ops.mesh.primitive_cube_add()
    #bpy.ops.object.delete() ## this can also disactive object

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_pattern(pattern=childName)
    bpy.context.scene.objects.active=bpy.context.scene.objects[childName]
    bpy.ops.object.select_pattern(pattern=parentName)
    bpy.context.scene.objects.active=bpy.context.scene.objects[parentName]##in(2.8) is :bpy.context.view_layer.objects.active
    bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)
    bpy.ops.object.select_all(action='DESELECT')


        
def viewsAdd(fileName = fileNameIn, scale =0.45 ):

    for i in bpy.data.objects:
        if((i.name[:6]=="Camera") or (i.name[:3]=="zqr")):
            bpy.ops.object.select_all(action='DESELECT')
            bpy.ops.object.select_pattern(pattern=i.name)
            bpy.ops.object.delete()


    matrix = np.loadtxt(fileName, usecols=range(6))
    #print(matrix, matrix.size, matrix.shape)

    basicAdd(part ="Cube" , name = "CameraCollection", location =(-0.02207, 0.08705, 0.61401), rotation = (0, 0, 0), scale =(0.01, 0.01, 0.01) )

    ##  
    for i in range(matrix.size):
        bpy.ops.object.camera_add(view_align=True, enter_editmode=False, location=(matrix[i][0], matrix[i][1], matrix[i][2]), rotation=(1.578-matrix[i][4], 0+matrix[i][5], -1.578+matrix[i][3] ), layers=(True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False))
        parentObjects("Camera", "CameraCollection")
        bpy.data.objects["Camera"].scale =(scale, scale, scale)
        bpy.data.objects["Camera"].name = "Camera"+str(i)

        #matrix[i][3], matrix[i][4], matrix[i][5]
        # yaw, pitch, row.
        # we apply 1.578, 0, -1.578, then 
        # the [rx, ry, rz] mapsto [pitch, row, yaw]
        
        #1.578, 0, -1.578
        #-1 *, -1* , 


if __name__ == "__main__":

    viewsAdd(scale = 0.3)
    

    
