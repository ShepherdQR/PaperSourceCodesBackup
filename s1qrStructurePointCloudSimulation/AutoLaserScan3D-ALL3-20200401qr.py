'''
@Author: Shepherd Qirong
@Date: 2020-04-01 23:06:10
@Github: https://github.com/ShepherdQR
@LastEditors: Shepherd Qirong
@LastEditTime: 2020-05-23 15:05:40
@Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
'''



bl_info = {
    "name": "Auto Scan with Laser",
    "author": "Qirong ZHANG",
    "version": (0, 0, 1),
    "blender": (2, 79, 4),
    "location": "",
    #"View3D > Add > Curve > Extra Objects",
    "description": "Auto scan with 2D Laser",
    "warning": "",
    "wiki_url": "",
    "category": "Auto Scan 3D to pointcloud",
    "time": "20200401"
}


import bpy, time, os
import numpy as np
import blensor


scanSwitch = True

# model 
whichModel = 2
nameModel = ("01Tank-T99A-20200322", "02Tank-2A5-20200522", "03Tank-T90A-20200522")
lengthModel = (8.0, 5.3, 7.2 )
slideMoedel = (800, 265, 400 )

oriZModel = (0, 0.36, 0)


# camera
### camera: half angle 27.8degree, angle_resolution = 2*27.8/1920, we set to k, for example 2*27.8/40, 
halfAngle = 27.8
AngleResolution = 2*halfAngle/1920
AngleResolution = 2*halfAngle/60
AngleResolution = 2*halfAngle/(1920/4)
maxRadius = 2.125

pi = 3.141592654
timeCur = time.strftime("%Y%m%d%H%M%S", time.localtime())
time_4=int(timeCur[-4:])
timeMoment = time.time() # global time to define preiods

modlePath = "/home/jellyfish/datasets/bunny/reconstruction/bun_zipper1889.ply"

projectPath = "/home/jellyfish/cpps/FinalPaperSourceCodes/AutoScanProject-20200324"


def outLog(line = "\n", timePassed= -1, folder =projectPath+"/scanFiles/"+timeCur, logName = timeCur + ".log", cover = False):
    ## usage outLog("hi", 1)// with time

    #logNameFront = logName.split('.')[0]
    if not os.path.exists(folder):
        os.makedirs(folder)
    
    switch1 = 'a'
    if(cover == True):
        switch1 = 'w+'

    with open(folder + "/"+logName, switch1) as f:
        f.write(line+"\n")
        if(timePassed != -1):
            time2 = time.time()
            global timeMoment
            time1 = timeMoment
            timePeriod = (time2 - time1) * 1000 #ms
            timeMoment = time2
            f.write("Process time range: [" +str(time1) +"] to [" +str(time2) +"], time Pariod is " + str(timePeriod) + " ms.\n")

def basicAdd( part ="Cube" , name = "DefaultN", location =(0, 0, 0), scale =(0.1, 0.1, 0.1)  ):    
    if( part == "Cube"  ):
        bpy.ops.mesh.primitive_cube_add()
    if ( part == "Cone"  ):
        bpy.ops.mesh.primitive_cone_add()
    obj_j = bpy.data.objects[part]
    obj_j.location = location
    obj_j.scale =  scale
    obj_j.rotation_euler = (0, 0, 0)
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


# ####### ------------ structure---------

def End(  ):
    # using the default scenes
    #samples = 64

    timeStart = time.strftime("%Y%m%d%H%M%S", time.localtime())
    outLog(line = "// time \n"+ timeStart +"\n", timePassed= -1, folder =projectPath+"/commonSettings/", logName =  "Settings.log", cover = True)
    outLog("Ends at "+ timeStart +"\n")


def Initialization( device = "GPU", samples = 64 ):
    # using the default scenes
    #samples = 64

    timeStart = time.strftime("%Y%m%d%H%M%S", time.localtime())
    outLog(line = "// time \n"+ timeStart +"\n", timePassed= -1, folder =projectPath+"/commonSettings/", logName =  "Settings.log", cover = True)
    outLog("Starts at "+ timeStart +"\n")

    bpy.data.scenes['Scene'].unit_settings.system = 'METRIC'
    bpy.data.scenes['Scene'].unit_settings.scale_length = 1
    bpy.data.scenes['Scene'].unit_settings.system_rotation = 'DEGREES'
    bpy.data.scenes['Scene'].render.engine = 'BLENDER_RENDER'
    
    bpy.context.scene.cycles.device = device
    bpy.context.scene.cycles.samples = samples
    resolution_range_x=[ 108*10, 1090 ]
    resolution_range_y = [ 72*10   ,1080   ]
    nnrr = 1
    bpy.context.scene.render.resolution_x = resolution_range_x[ nnrr ]
    bpy.context.scene.render.resolution_y = resolution_range_y[nnrr]
    bpy.context.scene.render.resolution_percentage = 100
    bpy.data.scenes['Scene'].frame_start = 1
    bpy.data.scenes['Scene'].frame_end = 800



    bpy.data.objects[nameModel[whichModel]].location =[0,0,oriZModel[whichModel]]
    for i in bpy.data.objects:
        if((i.name[:4]=="Scan") or (i.name[:3]=="zqr")):
            bpy.ops.object.select_all(action='DESELECT')
            bpy.ops.object.select_pattern(pattern=i.name)
            bpy.ops.object.delete()
    outLog("System setting ends ", 1)



def tankMoving( dy = 0.1):
    bpy.data.objects[nameModel[whichModel]].location[1] +=dy 

    
def scan2DLaser( cameraMode = 0, save2File = 1, showScan = True, frame = 0, dy = 0.1):
    camList = ["Linear00SL", "Linear01TL", "Linear02TR", "Linear03SR" ]
    modeList = [[3], [3, 0, 2, 1]]
    
    for i in range(len( modeList[cameraMode] )):
        camCurrent = camList[ modeList[cameraMode][i] ]# name
        camCurrent_ = bpy.data.objects[camCurrent]
        #camCurrent_.hide = False
        #camCurrent_.hide = True
        bpy.data.scenes["Scene"].camera  = camCurrent_
        

        scanCollectionCurrent = "Scan"+ camCurrent[-4:]
        outLog(line = "Scan log...\n", timePassed= 1, folder =projectPath+"/scanFiles/"+timeCur+"/"+scanCollectionCurrent, logName = scanCollectionCurrent+".log")

        #basicAdd( part ="Cone" , name = scanCollectionCurrent, location =(0, 0.4+0.4, 0), scale =(0.1, 0.1, 0.1)  )

        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_pattern(pattern=camCurrent )
        bpy.context.scene.objects.active=bpy.context.scene.objects[camCurrent]
        


        #basicAdd( part ="Cone" , name = scanCollectionCurrent, location =(0, 0.4+0.4, 0), scale =(0.1, 0.1, 0.1)  )
        
        scanSavePath = projectPath+"/scanFiles/"+timeCur+"/"+scanCollectionCurrent+ "/"+scanCollectionCurrent+str(frame)+ ".pcd"
        #
        
        camCurrent_.generic_angle_resolution =AngleResolution
        camCurrent_.generic_max_dist = maxRadius
        camCurrent_.generic_start_angle = -halfAngle
        camCurrent_.generic_end_angle = halfAngle
        
        bpy.ops.blensor.scan(filepath=scanSavePath)
        #blensor.generic_lidar.scan_advanced(scanner_object= camCurrent_,  evd_file=scanSavePath,add_blender_mesh = True, add_noisy_blender_mesh = False)

        if(showScan == False):
            bpy.ops.blensor.delete_scans()
        if(showScan ==True):
            if(frame == 0):
                basicAdd( part ="Cube" , name = scanCollectionCurrent, location =(0, 0.4+0.4*i, 0), scale =(0.1, 0.1, 0.1)  )
                bpy.data.objects[scanCollectionCurrent].hide = True
                lockfunction( part = scanCollectionCurrent, lock =True )
                parentObjects(scanCollectionCurrent, nameModel[whichModel])

            for i in bpy.data.objects:
                if((i.name[:5]=="Scan.") and (len(i.name)< 10)):
                    ## the second function is useless...
                    #ScanOriName = i.name
                    i.name = "zqr"+ scanCollectionCurrent +"-" + str(frame)
     
    if(showScan ==True):
        ## I am very upset about this part, I tried many times using the parenting function, but it did not work out. I CAN NOT select the Scan.xxx object. Actually I think the parenting thing cost me about 8 or more hours.
        for i in bpy.data.objects:
            if((i.name[:3]=="zqr") and (i.name.split('-')[1] != str(frame))):
                i.location[1] +=dy 


# =========== register =============== starts...
class AutoScanLaser(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "object.auto_scan_laser"
    bl_label = "AutoScan_PointCloudXYZ"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):
        bpy.ops.screen.frame_jump(end=False)

        print_bojects(context)
        return {'FINISHED'}


addon_keymaps = []
classes = [AutoScanLaser]


def menu_func1(self, context):
        self.layout.operator(AutoScanLaser.bl_idname)


def register1():
    bpy.utils.register_class(AutoScanLaser)
    bpy.types.VIEW3D_MT_object.append(menu_func1)
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if kc:
        km = wm.keyconfigs.addon.keymaps.new(name='Object Mode', space_type='EMPTY')
        kmi = km.keymap_items.new(AutoScanLaser.bl_idname, 'M', 'PRESS',   ctrl=True,alt=True)#shift=True,
        #kmi.properties.total = 4
        addon_keymaps.append((km, kmi))


def unregister1():
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()
    bpy.utils.unregister_class(AutoScanLaser)
    bpy.types.VIEW3D_MT_object.remove(menu_func1)

# =========== register =============== ends...




if __name__ == "__main__":

    # timeStart = time.strftime("%Y%m%d%H%M%S", time.localtime())
    # outLog("Starts at "+ timeStart +"\n")



    Initialization( device = "GPU", samples = 64 )

    if scanSwitch:
        for i in range( slideMoedel[whichModel] ):
            speed = lengthModel[whichModel] /  slideMoedel[whichModel]    #m/s
            tankMoving( dy = speed)
            scan2DLaser( cameraMode = 1, save2File = 1, showScan = True, frame = i, dy = speed )
        
    End()


    #scan2DLaser( cameraMode = 0, save2File = 1)
    #register1()
