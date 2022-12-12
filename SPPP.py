#coding=utf-8
##      Stiffness Prediction for Periodic Plate (SPPP) Ver. 1.0   (01/08/2022) 
##      SPPP is an ABAQUS CAE plugin developed to estimate the stiffness of periodic plate 
##      Copyright (C) 2022  Dajiang Wu
##      This is the first time for the author to publish a auxiliary tools on an open platform. Please forgive me if there are any irregularities.
##      Please contact the author if you have any bugs Email tlbsjagk@sina.com

### Importing ABAQUS Data and Python modules ##

from abaqus import *
from abaqusConstants import *
import __main__
import math
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
import time
import os
import sys
import ctypes
import multiprocessing
import re

## Plugin main GUI function ##

def npt(part,inst,meshsens,mode1,mode2,mode3,mode4,mode5,mode6,mode7,mode8,CPU,onlyPBC):
    import os
    path = os.getcwd()
    for T in (range(1)):
        start = time.time()
        modelName = part
        instanceName = inst
        upperName= inst.upper()
        
        fail = []
        keycheck2 =[inst]
        
        if part not in (mdb.models.keys()):
                Er2=0
                messageBox2 = ctypes.windll.user32.MessageBoxA
                returnValue = messageBox2(Er2,'Model name is incorrect, please input the correct Model name.','EasyPBC Start-up error 02',0x30 | 0x0)
                print('Start-up error 02. Refer EasyPBC user guide')
                continue
        
        a = mdb.models[modelName].rootAssembly
        errorcheck1 = mdb.models[modelName].rootAssembly.instances.keys()
        if errorcheck1 == fail:
                Er1=0
                messageBox1 = ctypes.windll.user32.MessageBoxA
                returnValue = messageBox1(Er1,'Model part is not created!\nPlease create part and try again','EasyPBC Start-up error 01',0x30 | 0x0)
                print('Start-up error 01. Refer EasyPBC user guide')
                continue
        
        if (mdb.models[modelName].rootAssembly.instances.keys()) != keycheck2:                       
                Er3=0
                messageBox3 = ctypes.windll.user32.MessageBoxA
                returnValue = messageBox3(Er3,'Instance name is incorrect, please input the correct instance name.','EasyPBC Start-up error 03',0x30 | 0x0)
                print('Start-up error 03. Refer EasyPBC user guide')
                continue

        if CPU <= 0:
                Er5=0
                messageBox5 = ctypes.windll.user32.MessageBoxA
                returnValue = messageBox5(Er5,'Specified number of CPUs is <= zero, please set it to a value larger than zero.','EasyPBC Start-up error 05',0x30 | 0x0)
                print('Start-up error 05. Refer EasyPBC user guide')
                continue

        
        CPUs = int(round(CPU))
        if CPUs > multiprocessing.cpu_count():
                CPUs = multiprocessing.cpu_count()
                print ('Warning: Specified number of CPUs is greater than the available. The maximum available number of CPUs is used (%s CPU(s)).' % CPUs)


        Nodeset = mdb.models[modelName].rootAssembly.instances[instanceName].nodes


############### Start of sets creation #############################################################################################################################################################

        j = 0
        x=[]
        y=[]
        z=[]
        c1=[]
        c2=[]
        c3=[]
        c4=[]
        c5=[]
        c6=[]
        c7=[]
        c8=[]
        Max=[]
        ftedgexyz={}
        btedgexyz={}
        fbedgexyz={}
        bbedgexyz={}
        fledgexyz={}
        bledgexyz={}
        fredgexyz={}
        bredgexyz={}
        ltedgexyz={}
        rtedgexyz={}
        lbedgexyz={}
        rbedgexyz={}
        frontsxyz={}
        backsxyz={}
        topsxyz={}
        botsxyz={}
        leftsxyz={}
        rightsxyz={}
        frontbcxyz={}
        backbcxyz={}
        topbcxyz={}
        botbcxyz={}
        leftbcxyz={}
        rightbcxyz={}
        cubexyz={}
        shellxyz={}
        shell=[]
        cube=[]
        shellterm=[]
        ftedge=[]
        btedge=[]
        fbedge=[]
        bbedge=[]
        fledge=[]
        fredge=[]
        bledge=[]
        bredge=[]
        ltedge=[]
        lbedge=[]
        rtedge=[]
        rbedge=[]
        fronts=[]
        backs=[]
        lefts=[]
        rights=[]
        tops=[]
        bots=[]
        backs=[]
        frontbc=[]
        backbc=[]
        leftbc=[]
        rightbc=[]
        topbc=[]
        botbc=[]
        backbc=[]
        errorset=[]
        coc1={}
        coc2={}
        coc3={}
        coc4={}
        coc5={}
        coc6={}
        coc7={}
        coc8={}

        error=False


        print ('----------------------------------')
        print ('-------- Start of EasyNPT --------')
        print ('----------------------------------')



        ## Identifying RVE size ##    
        for i in Nodeset:
            x.insert(j,i.coordinates[0])
            y.insert(j,i.coordinates[1])
            z.insert(j,i.coordinates[2])
            j=j+1



        errorcheck4 = x                   
        if not errorcheck4:
                Er4=0
                messageBox4 = ctypes.windll.user32.MessageBoxA
                returnValue = messageBox4(Er4,'Instance not detected! Make sure:\n1- Instance is created;\n2- Double click on instance to refresh it before running EasyPBC;\n3- Part/instnace is meshed.','EasyPBC Start-up error 04',0x30 | 0x0)
                print('Start-up error 04. Refer EasyPBC user guide')
                continue

        Max = max(x)
        May = max(y)
        Maz = max(z)
        Mnx = min(x)
        Mny = min(y)
        Mnz = min(z)

        L=abs(Max-Mnx)
        H=abs(May-Mny)
        W=abs(Maz-Mnz)
        
        Dispx = L
        Dispy = H
        Dispz = W

        ## Creating the face_set #########################################################

        a = mdb.models[modelName].rootAssembly
        v1 = a.instances[inst].nodes
        I = mdb. models[modelName]. rootAssembly.instances[inst];
        a = mdb.models[modelName].rootAssembly
        v1 = a.instances[inst].vertices
        f1 = a.instances[inst].faces
        e1 = a.instances[inst].edges

        # Faces Z- e Z+
        #
        faces1 = f1.getByBoundingBox(Mnx-meshsens,Mny-meshsens, Mnz-meshsens, Max+meshsens, May+meshsens,Mnz+meshsens)
        a.Set(faces=faces1, name='face_Z-')
        faces1 = f1.getByBoundingBox(Mnx-meshsens,Mny-meshsens, Maz-meshsens, Max+meshsens, May+meshsens,Maz+meshsens)
        a.Set(faces=faces1, name='face_Z+')
        #
        # Faces Y- e Y+
        #
        faces1 = f1.getByBoundingBox(Mnx-meshsens,Mny-meshsens, Mnz-meshsens, Max+meshsens, Mny+meshsens, Maz+meshsens)
        a.Set(faces=faces1, name='face_Y-')
        faces1 = f1.getByBoundingBox(Mnx-meshsens,May-meshsens, Mnz-meshsens, Max+meshsens, May+meshsens,Maz+meshsens)
        a.Set(faces=faces1, name='face_Y+')
        #
        # Faces X- e X+
        #
        faces1 = f1.getByBoundingBox(Mnx-meshsens,Mny-meshsens, Mnz-meshsens, Mnx+meshsens, May+meshsens, Maz+meshsens)
        a.Set(faces=faces1, name='face_X-')
        faces1 = f1.getByBoundingBox(Max-meshsens, Mny-meshsens, Mnz-meshsens, Max+meshsens, May+meshsens,Maz+meshsens)
        a.Set(faces=faces1, name='face_X+')
        #

############### Set the  Ref. Points################################################################################################################################################################
        ## Creating Ref. Points ##
        for i in a.features.keys():
            if i.startswith('RP'):
                del a.features['%s' % (i)]
        a.ReferencePoint(point=(Max+0.8*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP1:  X-axis
        a.ReferencePoint(point=(Max+0.6*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP2:  X-axis
        a.ReferencePoint(point=(Max+0.4*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP3:  X-axis
        a.ReferencePoint(point=(Max+0.2*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP4:  X-axis
        a.ReferencePoint(point=(Max-0.5*(Max-Mnx), May-0.5*(May-Mny), Maz+0.2*abs(Maz-Mnz)))  ## RP5:  Z-axis
        a.ReferencePoint(point=(Max-0.5*(Max-Mnx), May+1.0*abs(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP6:  Y-axis

        r1 = a.referencePoints

        ## Naming Ref. Points ##
        d=6
        for i in r1.keys():
            refPoints1=(r1[i], )
            a.Set(referencePoints=refPoints1, name='RP%s' % (d))
            d=d-1
          
        ## Identifying boundary nodes ##



############### make the density to unit mass  to get the consistent mass matrix####################################################################################################################
        if os.path.exists((path+'\mass.txt')):
            print ('The consistent-mass-Matrix is aready caculate')
            
        else:
            mdb.models['Model-1'].StaticStep(name='Step-mass', previous='Initial')
            session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-mass')
            
            
            mdb.Job(name='consistent-mass-Matrix', model='Model-1', description='', 
                type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, 
                memory=90, memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=1, 
                numGPUs=0)

            mdb.jobs['consistent-mass-Matrix'].submit(consistencyChecking=OFF)
            mdb.jobs['consistent-mass-Matrix'].waitForCompletion()

            inpfile=open((path+'\consistent-mass-Matrix.inp'))
            lines=inpfile.readlines()
            inpfile.close()
            laststr="*End Step\n"
            strindex=lines.index(laststr)
            lines.insert(strindex+1,"*STEP, NAME = MATRIX\n")
            lines.insert(strindex+2,"*MATRIX GENERATE, STIFFNESS, MASS\n")
            lines.insert(strindex+3,"*MATRIX OUTPUT, STIFFNESS, MASS\n")
            lines.insert(strindex+4,"*END STEP")
            newfile=open((path+'\consistent-mass-Matrix-new.inp'),"w")
            for newline in lines:
                newfile.write(newline)

            newfile.close()


            mdb.JobFromInputFile(name='consistent-mass-Matrix-new', 
                inputFileName=(path+'\consistent-mass-Matrix-new.inp'), 
                type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, 
                memory=90, memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, userSubroutine='', 
                scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=1, 
                numGPUs=0)

            mdb.jobs['consistent-mass-Matrix-new'].submit(consistencyChecking=OFF)


        ## Set the mass data of nodes for H11 H22 caculate  ##
        mass_file =  path+'\mass.txt'


############### Distinguish node between face and edge corner ######################################################################################################################################

        for i in Nodeset:
           if i.coordinates[2] != 0:
                cubexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]

        for i in Nodeset:
            if (Mnx+meshsens) < i.coordinates[0] < (Max-meshsens) and (Mny+meshsens) < i.coordinates[1] < (May-meshsens) and (Mnz+meshsens) < i.coordinates[2] < (Maz-meshsens):
                continue
            if abs(i.coordinates[0]-Max)<=meshsens:
                frontbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[0]-Mnx)<=meshsens:
                backbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[2]-Maz)<=meshsens:
                leftbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[2]-Mnz)<=meshsens:
                rightbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[1]-May)<=meshsens:
                topbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[1]-Mny)<=meshsens:
                botbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens:
                c1.insert(0,i.label)
                coc1[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens:
                c2.insert(0,i.label)
                coc2[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens:
                c3.insert(0,i.label)
                coc3[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens:
                c4.insert(0,i.label)
                coc4[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens:
                c5.insert(0,i.label)
                coc5[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens:
                c6.insert(0,i.label)
                coc6[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens:
                c7.insert(0,i.label)
                coc7[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens:
                c8.insert(0,i.label)
                coc8[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                ftedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                fbedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                btedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                bbedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                fledgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                fredgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                bledgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                bredgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                ltedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                lbedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                rtedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                rbedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                frontsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                backsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]] 
            if abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                leftsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
            if abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                rightsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]   
            if abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                topsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
            if abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                botsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]


############### Checking number of nodes of opposite/associated sets  ##############################################################################################################################

        if len(frontsxyz) != len(backsxyz):
            print ('Warning: Number of Nodes in Front surface (fronts) not equal to number of nodes in Back surface (backs). These sets will not be created!!')
            print ('         Refer to error 06 troubleshooting in easyPBC user guide.')
            frontsxyz={}
            error=True
        if len(topsxyz) != len(botsxyz):
            print ('Warning: Number of Nodes in Top surface (tops) not equal to number of nodes in Bottom surface (bots). These sets will not be created!!')
            print ('         Refer to error 06 in easyPBC user guide.')
            topsxyz={}
            error=True
        if len(leftsxyz) != len(rightsxyz):
            print ('Warning: Number of Nodes in Left surface (lefts) not equal to number of nodes in Right surface (rights). These sets will not be created!!')
            print ('         Refer to error 06 in easyPBC user guide.')
            leftsxyz={}
            error=True
        if len(ftedgexyz) != len(btedgexyz) or len(btedgexyz) != len(bbedgexyz) or len(bbedgexyz) != len(ftedgexyz):
            print ('Warning: Number of nodes in front-top ,back-top, back-bottom, front-bottom (ftedge, btedge, bbedge and fbedge) are not equal. These sets will not be created!!')
            print ('         Refer to error 06 in easyPBC user guide.')
            ftedgexyz={}
            error=True
        if len(fledgexyz) != len(bledgexyz) or len(bledgexyz) != len(bredgexyz) or len(bredgexyz) != len(fredgexyz):
            print ('Warning: Number of nodes in front-left, back-left, back-right, front-right edge (fledge, bledge, bredge and fredge) are not equal. These sets will not be created!!')
            print ('         Refer to error 06 in easyPBC user guide.')
            fledgexyz={}
            error=True
        if len(ltedgexyz) != len(rtedgexyz) or len(rtedgexyz) != len(rbedgexyz) or len(rbedgexyz) != len(lbedgexyz):
            print ('Warning: Number of nodes in left-top, right-top, right-bottom, front-bottom edge (ltedge, rtedge, rbedge and fbedge). are not equal. These sets will not be created!!')
            print ('         Refer to error 06 in easyPBC user guide.')
            ltedgexyz={}
            error=True
        if len(frontbcxyz) != len(backbcxyz):
            print ('Warning: Number of Nodes in Front BC surface (frontbc) not equal to number of nodes in Back BC surface (backbc). These sets will not be created!!')
            print ('         Refer to error 06 troubleshooting in easyPBC user guide.')
            frontbcxyz={}
            error=True
        if len(topbcxyz) != len(botbcxyz):
            print ('Warning: Number of Nodes in Top BC surface (topbc) not equal to number of nodes in Bottom BC surface (botbc). These sets will not be created!!')
            print ('         Refer to error 06 in easyPBC user guide.')
            topbcxyz={}
            error=True
        if len(leftbcxyz) != len(rightbcxyz):
            print 'Warning: Number of Nodes in Left BC surface (leftbc) not equal to number of nodes in Right BC surface (rightbc). These sets will not be created!!'
            print '         Refer to error 06 in easyPBC user guide.'
            leftbcxyz={}
            error=True

        ## Sorting and appending sets ##
        for i in frontsxyz.keys():
                for k in backsxyz.keys():
                        if abs(frontsxyz[i][1] - backsxyz[k][1])<=meshsens and abs(frontsxyz[i][2] - backsxyz[k][2])<=meshsens:
                                fronts.append(i)
                                backs.append(k)

        for i in cubexyz.keys():
                cube.append(i)


        if len(frontsxyz)!= len(fronts) or len(backsxyz)!= len(backs):
            print ('Warning: Node(s) in Front and/or Back surface (fronts and/or backs) was not imported. effected sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
            for i, k in zip(frontsxyz.keys(),backsxyz.keys()):
                    if i not in fronts:
                            errorset.append(i)
                    if k not in backs:
                            errorset.append(k)
            fronts=[]
            backs=[]
            error=True                    
        if len(fronts)!=len(set(fronts)) or len(backs)!=len(set(backs)):
            print ('Warning: Node(s) in either Front or Back surface (fronts or backs) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            fronts=[]
            backs=[]
            error=True

        for i in topsxyz.keys():
            for k in botsxyz.keys():
                if abs(topsxyz[i][0] - botsxyz[k][0]) <=meshsens and abs(topsxyz[i][2] - botsxyz[k][2]) <=meshsens:
                    tops.append(i)
                    bots.append(k)
        if len(topsxyz)!= len(tops) or len(botsxyz)!= len(bots):
            print ('Warning: Node(s) in Top and/or Bottom surface (tops and/or bots) was not imported. effected sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
            for i, k in zip(topsxyz.keys(),botsxyz.keys()):
                    if i not in tops:
                            errorset.append(i)
                    if k not in bots:
                            errorset.append(k)
            tops=[]
            bots=[]
            error=True
        if len(tops)!=len(set(tops)) or len(bots)!=len(set(bots)):
            print ('Warning: Node(s) in either Top or Bottom surface (tops or bots) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            tops=[]
            bots=[]
            error=True


        for i in leftsxyz.keys():
            for k in rightsxyz.keys():
                if abs(leftsxyz[i][0] - rightsxyz[k][0])<=meshsens and abs(leftsxyz[i][1] - rightsxyz[k][1]) <=meshsens:
                    lefts.append(i)
                    rights.append(k)
        if len(leftsxyz)!= len(lefts) or len(rightsxyz)!= len(rights):
            print ('Warning: Node(s) in Left and/or Right surface (lefts and/or rights) was not imported. effected sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
            for i, k in zip(leftsxyz.keys(),rightsxyz.keys()):
                    if i not in lefts:
                            errorset.append(i)
                    if k not in rights:
                            errorset.append(k)                    
            lefts=[]
            rights=[]
            error=True                    
        if len(lefts)!=len(set(lefts)) or len(rights)!=len(set(rights)):
            print ('Warning: Node(s) in either Left or Right surface (lefts or rights) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            lefts=[]
            rights=[]
            error=True

        for i in frontbcxyz.keys():
            for k in backbcxyz.keys():
                if abs(frontbcxyz[i][1] - backbcxyz[k][1])<=meshsens and abs(frontbcxyz[i][2] - backbcxyz[k][2])<=meshsens:
                    frontbc.append(i)
                    backbc.append(k)
        if len(frontbcxyz)!= len(frontbc) or len(backbcxyz)!= len(backbc):
            print ('Warning: Node(s) in Front BC and/or Back BC surface (frontbc and/or backbc) was not imported. effected sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
            for i, k in zip(frontbcxyz.keys(),backbcxyz.keys()):
                    if i not in frontbc:
                            errorset.append(i)
                    if k not in backbc:
                            errorset.append(k)
            frontbc=[]
            backbc=[]
            error=True
        if len(frontbc)!=len(set(frontbc)) or len(backbc)!=len(set(backbc)):
            print ('Warning: Node(s) in either Front BC or Back BC surface (frontbc or backbc) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            frontbc=[]
            backbc=[]
            error=True


        for i in topbcxyz.keys():
            for k in botbcxyz.keys():
                if abs(topbcxyz[i][0] - botbcxyz[k][0]) <=meshsens and abs(topbcxyz[i][2] - botbcxyz[k][2]) <=meshsens:
                    topbc.append(i)
                    botbc.append(k)
        if len(topbcxyz)!= len(topbc) or len(botbcxyz)!= len(botbc):
            print ('Warning: Node(s) in Top BC and/or Bottom BC surface (topbc and/or botbc) was not imported. effected sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
            for i, k in zip(topbcxyz.keys(),botbcxyz.keys()):
                    if i not in topbc:
                            errorset.append(i)
                    if k not in botbc:
                            errorset.append(k)
            topbc=[]
            botbc=[]
            error=True
        if len(topbc)!=len(set(topbc)) or len(botbc)!=len(set(botbc)):
            print ('Warning: Node(s) in either Top BC or Bottom BC surface (topbc or botbc) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            topbc=[]
            botbc=[]
            error=True


        for i in leftbcxyz.keys():
            for k in rightbcxyz.keys():
                if abs(leftbcxyz[i][0] - rightbcxyz[k][0])<=meshsens and abs(leftbcxyz[i][1] - rightbcxyz[k][1]) <=meshsens:
                    leftbc.append(i)
                    rightbc.append(k)
        if len(leftbcxyz)!= len(leftbc) or len(rightbcxyz)!= len(rightbc):
            print ('Warning: Node(s) in Left BC and/or Right BC surface (lefts and/or rights) was not imported. effected sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
            for i, k in zip(leftbcxyz.keys(),rightbcxyz.keys()):
                    if i not in leftbc:
                            errorset.append(i)
                    if k not in rightbc:
                            errorset.append(k)            
            leftbc=[]
            rightbc=[]
            error=True
        if len(leftbc)!=len(set(leftbc)) or len(rightbc)!=len(set(rightbc)):
            print ('Warning: Node(s) in either Left BC or Right BC surface (leftbc or rightbc) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            leftbc=[]
            rightbc=[]
            error=True


        for i in ftedgexyz.keys():
            for k in btedgexyz.keys():
                if abs(ftedgexyz[i][1] - btedgexyz[k][1])<=meshsens and abs(ftedgexyz[i][2] - btedgexyz[k][2])<=meshsens:
                    ftedge.append(i)
                    btedge.append(k)
        for i in btedge:
            for k in bbedgexyz.keys():
                if abs(btedgexyz[i][0] - bbedgexyz[k][0]) <=meshsens and abs(btedgexyz[i][2] - bbedgexyz[k][2]) <=meshsens:
                    bbedge.append(k)    
        for i in bbedge:
            for k in fbedgexyz.keys():
                if abs(bbedgexyz[i][1] - fbedgexyz[k][1]) <=meshsens and abs(bbedgexyz[i][2] - fbedgexyz[k][2]) <=meshsens:
                    fbedge.append(k) 
        if len(ftedge)!=len(set(ftedge)) or len(btedge)!=len(set(btedge)) or len(bbedge)!=len(set(bbedge)) or len(fbedge)!=len(set(fbedge)):
            print ('Warning: Node(s) in either front-top, back-top, back-bottom and front-bottom edge(ftedge, btedge, bbedge and fbedge) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            ftedge=[]
            btedge=[]
            bbedg=[]
            fbedge=[]
            error==True
        if len(ftedgexyz)!= len(ftedge) or len(btedgexyz)!= len(btedge) or len(bbedgexyz)!= len(bbedge) or len(fbedgexyz)!= len(fbedge):
            print ('Warning: Node(s) in front-top, back-top, back-bottom and front-bottom edge(ftedge, btedge, bbedge and fbedge) were not imported. these sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
            ftedge=[]
            btedge=[]
            bbedg=[]
            fbedge=[]
            error=True

        for i in ltedgexyz.keys():
            for k in rtedgexyz.keys():
                if abs(ltedgexyz[i][0] - rtedgexyz[k][0])<=meshsens and abs(ltedgexyz[i][1] - rtedgexyz[k][1])<=meshsens:
                    ltedge.append(i)
                    rtedge.append(k)
        for i in rtedge:
            for k in rbedgexyz.keys():
                if abs(rtedgexyz[i][0] - rbedgexyz[k][0])<=meshsens and abs(rtedgexyz[i][2] - rbedgexyz[k][2])<=meshsens:
                    rbedge.append(k)    
        for i in rbedge:
            for k in lbedgexyz.keys():
                if abs(rbedgexyz[i][0] - lbedgexyz[k][0])<=meshsens and abs(rbedgexyz[i][1] - lbedgexyz[k][1])<=meshsens:
                    lbedge.append(k) 

        if len(ltedge)!=len(set(ltedge)) or len(rtedge)!=len(set(rtedge)) or len(rbedge)!=len(set(rbedge)) or len(lbedge)!=len(set(lbedge)):
            print ('Warning: Node(s) in either front-top, back-bottom and front-bottom edge(ltedge, rtedge, rbedge and lbedge) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            ltedge=[]
            rtedge=[]
            rbedg=[]
            lbedge=[]
            error=True

        if len(ltedgexyz)!= len(ltedge) or len(rtedgexyz)!= len(rtedge) or len(rbedgexyz)!= len(rbedge) or len(lbedgexyz)!= len(lbedge):
            print ('Warning: Node(s) in left-top, right-top, right-bottom, left-bottom edge (ltedge, rtedge, rbedge and lbedge) were not imported. these sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
            ltedge=[]
            rtedge=[]
            rbedg=[]
            lbedge=[]
            error=True

        for i in fledgexyz.keys():
            for k in bledgexyz.keys():
                if abs(fledgexyz[i][1] - bledgexyz[k][1])<=meshsens and abs(fledgexyz[i][2] - bledgexyz[k][2])<=meshsens:
                    fledge.append(i)
                    bledge.append(k)
        for i in bledge:
            for k in bredgexyz.keys():
                if abs(bledgexyz[i][0] - bredgexyz[k][0])<=meshsens and abs(bledgexyz[i][1] - bredgexyz[k][1])<=meshsens:
                    bredge.append(k)    
        for i in bredge:
            for k in fredgexyz.keys():
                if abs(bredgexyz[i][1] - fredgexyz[k][1])<=meshsens and abs(bredgexyz[i][2] - fredgexyz[k][2])<=meshsens:
                    fredge.append(k) 

        if len(fledge)!=len(set(fledge)) or len(bledge)!=len(set(bledge)) or len(bredge)!=len(set(bredge)) or len(fredge)!=len(set(fredge)):
            print ('Warning: Node(s) in either front-left, back-left, back-right and front-right edge(fledge, bledge, bredge and fredge) being linked with more than one opposite node. effected sets will not be created!!')
            print ('         Refer to error 08 in easyPBC user guide.')
            fledge=[]
            bledge=[]
            bredg=[]
            fredge=[]
            error=True
        if len(fledgexyz)!= len(fledge) or len(bledgexyz)!= len(bledge) or len(bredgexyz)!= len(bredge) or len(fredgexyz)!= len(fredge):
            print ('Warning: Node(s) in front-left, back-left, back-right and front-right edge (fledge, bledge, bredge and fredge) were not imported. these sets will not be created!!')
            print ('         Refer to error 07 in easyPBC user user guide.')
            fledge=[]
            bledge=[]
            bredg=[]
            fredge=[]
            error=True


        print ('--------------end of check--------------')


############### Creating ABAQUS sets ###############################################################################################################################################################
        
        a.SetFromNodeLabels(name='c1', nodeLabels=((instanceName,c1),))
        a.SetFromNodeLabels(name='c2', nodeLabels=((instanceName,c2),))
        a.SetFromNodeLabels(name='c3', nodeLabels=((instanceName,c3),))
        a.SetFromNodeLabels(name='c4', nodeLabels=((instanceName,c4),))
        a.SetFromNodeLabels(name='c5', nodeLabels=((instanceName,c5),))
        a.SetFromNodeLabels(name='c6', nodeLabels=((instanceName,c6),))
        a.SetFromNodeLabels(name='c7', nodeLabels=((instanceName,c7),))
        a.SetFromNodeLabels(name='c8', nodeLabels=((instanceName,c8),))
        a.SetFromNodeLabels(name='ftedge', nodeLabels=((instanceName,ftedge),))
        a.SetFromNodeLabels(name='fbedge', nodeLabels=((instanceName,fbedge),))
        a.SetFromNodeLabels(name='btedge', nodeLabels=((instanceName,btedge),))
        a.SetFromNodeLabels(name='bbedge', nodeLabels=((instanceName,bbedge),))
        a.SetFromNodeLabels(name='fledge', nodeLabels=((instanceName,fledge),))
        a.SetFromNodeLabels(name='fredge', nodeLabels=((instanceName,fredge),))
        a.SetFromNodeLabels(name='bledge', nodeLabels=((instanceName,bledge),))
        a.SetFromNodeLabels(name='bredge', nodeLabels=((instanceName,bredge),))
        a.SetFromNodeLabels(name='ltedge', nodeLabels=((instanceName,ltedge),))
        a.SetFromNodeLabels(name='lbedge', nodeLabels=((instanceName,lbedge),))
        a.SetFromNodeLabels(name='rtedge', nodeLabels=((instanceName,rtedge),))
        a.SetFromNodeLabels(name='rbedge', nodeLabels=((instanceName,rbedge),))
        a.SetFromNodeLabels(name='fronts', nodeLabels=((instanceName,fronts),))
        a.SetFromNodeLabels(name='backs', nodeLabels=((instanceName,backs),))
        a.SetFromNodeLabels(name='lefts', nodeLabels=((instanceName,lefts),))
        a.SetFromNodeLabels(name='rights', nodeLabels=((instanceName,rights),))
        a.SetFromNodeLabels(name='tops', nodeLabels=((instanceName,tops),))
        a.SetFromNodeLabels(name='bots', nodeLabels=((instanceName,bots),))
        a.SetFromNodeLabels(name='frontbc', nodeLabels=((instanceName,frontbc),))
        a.SetFromNodeLabels(name='backbc', nodeLabels=((instanceName,backbc),))
        a.SetFromNodeLabels(name='leftbc', nodeLabels=((instanceName,leftbc),))
        a.SetFromNodeLabels(name='rightbc', nodeLabels=((instanceName,rightbc),))
        a.SetFromNodeLabels(name='topbc', nodeLabels=((instanceName,topbc),))
        a.SetFromNodeLabels(name='botbc', nodeLabels=((instanceName,botbc),))
        a.SetFromNodeLabels(name='cube', nodeLabels=((instanceName,cube),))

        print ('------ End of Sets Creation ------')

############### Extracting model mass ##############################################################################################################################################################
        prop=mdb.models[modelName].rootAssembly.getMassProperties()
        mass=prop['mass']

        a = mdb.models[modelName].rootAssembly
        Nodeset = mdb.models[modelName].rootAssembly.instances[instanceName].nodes
        mdb.models[modelName].StaticStep(name='Step-1', previous='Initial')


############### Creating single-node ABAQUS sets ###################################################################################################################################################
        if error==False:
            print '------Start to set Contraints------'

            if mode1==True or A22==True or A66==True or D11==True or D22==True or D66==True or H11==True or H22==True:
                    for i,k in zip(tops,bots):
                        a.SetFromNodeLabels(name='tops%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='bots%s' % (k), nodeLabels=((instanceName,[k]),))

                    for i,k in zip(fronts,backs):
                        a.SetFromNodeLabels(name='fronts%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='backs%s' % (k), nodeLabels=((instanceName,[k]),))

                    for i,k in zip(lefts,rights):
                        a.SetFromNodeLabels(name='lefts%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='rights%s' % (k), nodeLabels=((instanceName,[k]),))

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        a.SetFromNodeLabels(name='ftedge%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='btedge%s' % (k), nodeLabels=((instanceName,[k]),))
                        a.SetFromNodeLabels(name='bbedge%s' % (j), nodeLabels=((instanceName,[j]),))
                        a.SetFromNodeLabels(name='fbedge%s' % (l), nodeLabels=((instanceName,[l]),))

                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                        a.SetFromNodeLabels(name='fledge%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='bledge%s' % (k), nodeLabels=((instanceName,[k]),))
                        a.SetFromNodeLabels(name='bredge%s' % (j), nodeLabels=((instanceName,[j]),))
                        a.SetFromNodeLabels(name='fredge%s' % (l), nodeLabels=((instanceName,[l]),))

                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                        a.SetFromNodeLabels(name='ltedge%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='lbedge%s' % (k), nodeLabels=((instanceName,[k]),))
                        a.SetFromNodeLabels(name='rbedge%s' % (j), nodeLabels=((instanceName,[j]),))
                        a.SetFromNodeLabels(name='rtedge%s' % (l), nodeLabels=((instanceName,[l]),))

                    for i,k in zip(topbc,botbc):
                        a.SetFromNodeLabels(name='topbc%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='botbc%s' % (k), nodeLabels=((instanceName,[k]),))

                    for i,k in zip(frontbc,backbc):
                        a.SetFromNodeLabels(name='frontbc%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='backbc%s' % (k), nodeLabels=((instanceName,[k]),))

                    for i,k in zip(leftbc,rightbc):
                        a.SetFromNodeLabels(name='leftbc%s' % (i), nodeLabels=((instanceName,[i]),))
                        a.SetFromNodeLabels(name='rightbc%s' % (k), nodeLabels=((instanceName,[k]),))

                    for i in (cube):
                        a.SetFromNodeLabels(name='cube%s' % (i), nodeLabels=((instanceName,[i]),))


####********************************* Calculate the equivalent stiffness coefficient by deformation mode1 ***********************************************************************************************************
            if mode1==True :
                    for i in mdb.models[modelName].constraints.keys():
                            del mdb.models[modelName].constraints[i]

                    ##############-----X-direction--face---##############
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A11-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1),(-1.0, 'RP4', 1)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A11-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A11-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3)))
                    ##############-----Y-direction--face---##############
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A11-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A11-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A11-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3)))

                    ##############-----X-direction--vertex---##############
                    mdb.models[modelName].Equation(name='A11-1-c12', terms=((1.0, 'c1', 1), (-1.0, 'c2', 1),(-1.0, 'RP4', 1)))
                    mdb.models[modelName].Equation(name='A11-1-c43', terms=((1.0, 'c4', 1), (-1.0, 'c3', 1),(-1.0, 'RP4', 1)))
                    mdb.models[modelName].Equation(name='A11-1-c56', terms=((1.0, 'c5', 1), (-1.0, 'c6', 1),(-1.0, 'RP4', 1)))
                    mdb.models[modelName].Equation(name='A11-1-c87', terms=((1.0, 'c8', 1), (-1.0, 'c7', 1),(-1.0, 'RP4', 1)))
                    mdb.models[modelName].Equation(name='A11-2-c56', terms=((1.0, 'c5', 2), (-1.0, 'c6', 2)))
                    mdb.models[modelName].Equation(name='A11-2-c87', terms=((1.0, 'c8', 2), (-1.0, 'c7', 2)))
                    mdb.models[modelName].Equation(name='A11-3-c56', terms=((1.0, 'c5', 3), (-1.0, 'c6', 3)))
                    mdb.models[modelName].Equation(name='A11-3-c87', terms=((1.0, 'c8', 3), (-1.0, 'c7', 3)))

                    ##############-----Y-direction--vertex---##############
                    mdb.models[modelName].Equation(name='A11-2-c26', terms=((1.0, 'c2', 2), (-1.0, 'c6', 2)))
                    mdb.models[modelName].Equation(name='A11-3-c26', terms=((1.0, 'c2', 3), (-1.0, 'c6', 3)))
                    mdb.models[modelName].Equation(name='A11-2-c15', terms=((1.0, 'c1', 2), (-1.0, 'c5', 2)))
                    mdb.models[modelName].Equation(name='A11-3-c15', terms=((1.0, 'c1', 3), (-1.0, 'c5', 3)))
                    mdb.models[modelName].Equation(name='A11-2-c48', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2)))
                    mdb.models[modelName].Equation(name='A11-3-c48', terms=((1.0, 'c4', 3), (-1.0, 'c8', 3)))
                    mdb.models[modelName].Equation(name='A11-2-c37', terms=((1.0, 'c3', 2), (-1.0, 'c7', 2)))
                    mdb.models[modelName].Equation(name='A11-3-c37', terms=((1.0, 'c3', 3), (-1.0, 'c7', 3)))

                    ##############-----X-direction--edge---##############
                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='A11-1-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 1), (-1.0, 'btedge%s'%k, 1),(-1.0, 'RP4', 1)))
                        mdb.models[modelName].Equation(name='A11-1-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 1), (-1.0, 'bbedge%s'%j, 1),(-1.0, 'RP4', 1)))
                        mdb.models[modelName].Equation(name='A11-2-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 2), (-1.0, 'btedge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='A11-2-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 2), (-1.0, 'bbedge%s'%j, 2)))
                        mdb.models[modelName].Equation(name='A11-3-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'btedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='A11-3-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 3), (-1.0, 'bbedge%s'%j, 3)))

                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                        mdb.models[modelName].Equation(name='A11-1-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 1), (-1.0, 'bledge%s'%k, 1),(-1.0, 'RP4', 1)))
                        mdb.models[modelName].Equation(name='A11-1-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 1), (-1.0, 'bredge%s'%j, 1),(-1.0, 'RP4', 1)))
                        mdb.models[modelName].Equation(name='A11-2-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 2), (-1.0, 'bledge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='A11-2-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 2), (-1.0, 'bredge%s'%j, 2)))
                        mdb.models[modelName].Equation(name='A11-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='A11-3-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 3), (-1.0, 'bredge%s'%j, 3)))

                    ##############-----Y-direction--edge---##############
                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                        mdb.models[modelName].Equation(name='A11-1-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 1), (-1.0, 'lbedge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='A11-1-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 1), (-1.0, 'rbedge%s'%j, 1)))  
                        mdb.models[modelName].Equation(name='A11-2-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 2), (-1.0, 'lbedge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='A11-2-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 2), (-1.0, 'rbedge%s'%j, 2)))  
                        mdb.models[modelName].Equation(name='A11-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='A11-3-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 3), (-1.0, 'rbedge%s'%j, 3)))  

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='A11-2-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 2), (-1.0, 'bbedge%s'%j, 2)))
                        mdb.models[modelName].Equation(name='A11-3-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 3), (-1.0, 'bbedge%s'%j, 3)))


            if mode1==True and onlyPBC == False:
                    for i in mdb.models[modelName].loads.keys():
                            del mdb.models[modelName].loads[i]
                    for i in mdb.models[modelName].boundaryConditions.keys():
                            del mdb.models[modelName].boundaryConditions[i]            


                    region = a.sets['RP4']
                    mdb.models[modelName].DisplacementBC(name='A11-1', createStepName='Step-1', 
                        region=region, u1=Dispx, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
                        createStepName='Step-1', variables=('NFORC', ))


                    mdb.Job(name='job-mode1', model= modelName, description='', type=ANALYSIS, 
                        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                        scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                    mdb.jobs['job-mode1'].submit(consistencyChecking=OFF)
                    mdb.jobs['job-mode1'].waitForCompletion()
                    o3 = session.openOdb(name='%s' % (path+'\job-mode1.odb'))
                    import os, glob                 
                    odb = session.odbs['%s' % (path+'\job-mode1.odb')]

                    ##############-----Caculate-for-A11---##############

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                        NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP4', ))

                    forceA11 = 0
                    for i in session.xyDataObjects.keys():
                        forceA11=forceA11+(session.xyDataObjects[i][0][1])
                    A11 = forceA11/H       


                    ##############-----Caculate-for-A12---##############

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]


                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_Y+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("TOPBC", ))

                    session.FreeBodyFromNodesElements(name='FreeBody-1', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)

                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-1', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceA12 = 0
                    for i in session.xyDataObjects.keys():
                        forceA12=forceA12+(session.xyDataObjects[i][0][1])
                    A12 = forceA12/H       


            if mode1==False or onlyPBC == True:
                    A11='N/A'
                    A12='N/A'

####********************************* Calculate the equivalent stiffness coefficient by deformation mode2***********************************************************************************************************

            if mode2==True:
                    for i in mdb.models[modelName].constraints.keys():
                            del mdb.models[modelName].constraints[i]

                    ##############-----Y-direction--face---##############
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A22-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A22-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2),(-1.0, 'RP6', 2)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A22-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3)))
                    ##############-----X-direction--face---##############
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A22-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A22-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A22-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3)))

                    ##############-----X-direction--vertex---##############
                    mdb.models[modelName].Equation(name='A22-1-c12', terms=((1.0, 'c1', 1), (-1.0, 'c2', 1)))
                    mdb.models[modelName].Equation(name='A22-1-c43', terms=((1.0, 'c4', 1), (-1.0, 'c3', 1)))
                    mdb.models[modelName].Equation(name='A22-1-c56', terms=((1.0, 'c5', 1), (-1.0, 'c6', 1)))
                    mdb.models[modelName].Equation(name='A22-1-c87', terms=((1.0, 'c8', 1), (-1.0, 'c7', 1)))
                    mdb.models[modelName].Equation(name='A22-3-c12', terms=((1.0, 'c1', 3), (-1.0, 'c2', 3)))
                    mdb.models[modelName].Equation(name='A22-3-c43', terms=((1.0, 'c4', 3), (-1.0, 'c3', 3)))
                    mdb.models[modelName].Equation(name='A22-3-c56', terms=((1.0, 'c5', 3), (-1.0, 'c6', 3)))
                    mdb.models[modelName].Equation(name='A22-3-c87', terms=((1.0, 'c8', 3), (-1.0, 'c7', 3)))
                    ##############-----Y-direction--vertex---##############
                    mdb.models[modelName].Equation(name='A22-2-c26', terms=((1.0, 'c2', 2), (-1.0, 'c6', 2),(-1.0, 'RP6', 2)))
                    mdb.models[modelName].Equation(name='A22-2-c15', terms=((1.0, 'c1', 2), (-1.0, 'c5', 2),(-1.0, 'RP6', 2)))
                    mdb.models[modelName].Equation(name='A22-2-c48', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2),(-1.0, 'RP6', 2)))
                    mdb.models[modelName].Equation(name='A22-2-c37', terms=((1.0, 'c3', 2), (-1.0, 'c7', 2),(-1.0, 'RP6', 2))) 

                    ##############-----X-direction--edge---##############
                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='A22-1-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 1), (-1.0, 'btedge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='A22-3-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'btedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='A22-1-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 1), (-1.0, 'bbedge%s'%j, 1)))
                        mdb.models[modelName].Equation(name='A22-3-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 3), (-1.0, 'bbedge%s'%j, 3)))

                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                        mdb.models[modelName].Equation(name='A22-1-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 1), (-1.0, 'bledge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='A22-1-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 1), (-1.0, 'bredge%s'%j, 1)))
                        mdb.models[modelName].Equation(name='A22-2-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 2), (-1.0, 'bledge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='A22-2-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 2), (-1.0, 'bredge%s'%j, 2)))
                        mdb.models[modelName].Equation(name='A22-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='A22-3-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 3), (-1.0, 'bredge%s'%j, 3)))

                    ##############-----Y-direction--edge---##############
                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                        mdb.models[modelName].Equation(name='A22-1-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 1), (-1.0, 'lbedge%s'%k, 1),(-1.0, 'RP6', 1)))
                        mdb.models[modelName].Equation(name='A22-1-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 1), (-1.0, 'rbedge%s'%j, 1),(-1.0, 'RP6', 1)))  
                        mdb.models[modelName].Equation(name='A22-2-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 2), (-1.0, 'lbedge%s'%k, 2),(-1.0, 'RP6', 2)))
                        mdb.models[modelName].Equation(name='A22-2-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 2), (-1.0, 'rbedge%s'%j, 2),(-1.0, 'RP6', 2)))  
                        mdb.models[modelName].Equation(name='A22-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3),(-1.0, 'RP6', 3)))
                        mdb.models[modelName].Equation(name='A22-3-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 3), (-1.0, 'rbedge%s'%j, 3),(-1.0, 'RP6', 3)))  

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='A22-2-ftedge-fbedge%s'%i, terms=((1.0, 'ftedge%s'%i, 2), (-1.0, 'fbedge%s'%l, 2),(-1.0, 'RP6', 2)))
                        mdb.models[modelName].Equation(name='A22-2-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 2), (-1.0, 'bbedge%s'%j, 2),(-1.0, 'RP6', 2)))


            if mode2==True and onlyPBC == False:
                    for i in mdb.models[modelName].loads.keys():
                            del mdb.models[modelName].loads[i]
                    for i in mdb.models[modelName].boundaryConditions.keys():
                            del mdb.models[modelName].boundaryConditions[i]            

                    region = a.sets['RP6']
                    mdb.models[modelName].DisplacementBC(name='A22-1', createStepName='Step-1', 
                        region=region, u1=UNSET, u2=Dispy, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
                        createStepName='Step-1', variables=('NFORC', ))


                    mdb.Job(name='job-mode2', model= modelName, description='', type=ANALYSIS, 
                        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                        scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                    mdb.jobs['job-mode2'].submit(consistencyChecking=OFF)
                    mdb.jobs['job-mode2'].waitForCompletion()
                    o3 = session.openOdb(name='%s' % (path+'\job-mode2.odb'))
                    import os, glob
                 
                    odb = session.odbs['%s' % (path+'\job-mode2.odb')]

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name

                    ##############-----Caculate-for-A22---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                        NODAL, ((COMPONENT, 'RF2'), )), ), nodeSets=('RP6', ))

                    forceA22 = 0
                    for i in session.xyDataObjects.keys():
                        forceA22=forceA22+(session.xyDataObjects[i][0][1])

                    A22 = forceA22/L                             

                    ##############-----Caculate-for-A21---##############

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]


                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_X+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("FRONTBC", ))

                    session.FreeBodyFromNodesElements(name='FreeBody-1', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-1', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceA21 = 0
                    for i in session.xyDataObjects.keys():
                        forceA21=forceA21+(session.xyDataObjects[i][0][1])
                    A21 = forceA21/L       


            if mode2==False or onlyPBC == True:
                    A22='N/A'
                    A21='N/A'

####********************************* Calculate the equivalent stiffness coefficient by deformation mode3 ***********************************************************************************************************

            if mode3==True:
                    for i in mdb.models[modelName].constraints.keys():
                            del mdb.models[modelName].constraints[i]

                    ##############-----X-direction--face---##############
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A66-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A66-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2),(-1.0, 'RP4', 2)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='A66-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3)))
                    ##############-----Y-direction--face---##############
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A66-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1),(-1.0, 'RP6', 1)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A66-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='A66-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3)))


                    ##############-----X-direction--vertex---##############

                    mdb.models[modelName].Equation(name='A66-1-c56', terms=((1.0, 'c5', 1), (-1.0, 'c6', 1)))
                    mdb.models[modelName].Equation(name='A66-1-c87', terms=((1.0, 'c8', 1), (-1.0, 'c7', 1)))
                    mdb.models[modelName].Equation(name='A66-2-c12', terms=((1.0, 'c1', 2), (-1.0, 'c2', 2),(-1.0, 'RP4', 2)))
                    mdb.models[modelName].Equation(name='A66-2-c43', terms=((1.0, 'c4', 2), (-1.0, 'c3', 2),(-1.0, 'RP4', 2)))
                    mdb.models[modelName].Equation(name='A66-2-c56', terms=((1.0, 'c5', 2), (-1.0, 'c6', 2),(-1.0, 'RP4', 2)))
                    mdb.models[modelName].Equation(name='A66-2-c87', terms=((1.0, 'c8', 2), (-1.0, 'c7', 2),(-1.0, 'RP4', 2)))

                    mdb.models[modelName].Equation(name='A66-3-c56', terms=((1.0, 'c5', 3), (-1.0, 'c6', 3)))
                    mdb.models[modelName].Equation(name='A66-3-c87', terms=((1.0, 'c8', 3), (-1.0, 'c7', 3)))
                    ##############-----Y-direction--vertex---##############
                    mdb.models[modelName].Equation(name='A66-1-c26', terms=((1.0, 'c2', 1), (-1.0, 'c6', 1),(-1.0, 'RP6', 1)))
                    mdb.models[modelName].Equation(name='A66-2-c26', terms=((1.0, 'c2', 2), (-1.0, 'c6', 2)))
                    mdb.models[modelName].Equation(name='A66-3-c26', terms=((1.0, 'c2', 3), (-1.0, 'c6', 3)))
                    mdb.models[modelName].Equation(name='A66-1-c15', terms=((1.0, 'c1', 1), (-1.0, 'c5', 1),(-1.0, 'RP6', 1)))
                    mdb.models[modelName].Equation(name='A66-3-c15', terms=((1.0, 'c1', 3), (-1.0, 'c5', 3)))
                    mdb.models[modelName].Equation(name='A66-1-c48', terms=((1.0, 'c4', 1), (-1.0, 'c8', 1),(-1.0, 'RP6', 1)))
                    mdb.models[modelName].Equation(name='A66-3-c48', terms=((1.0, 'c4', 3), (-1.0, 'c8', 3)))
                    mdb.models[modelName].Equation(name='A66-1-c37', terms=((1.0, 'c3', 1), (-1.0, 'c7', 1),(-1.0, 'RP6', 1)))
                    mdb.models[modelName].Equation(name='A66-2-c37', terms=((1.0, 'c3', 2), (-1.0, 'c7', 2)))
                    mdb.models[modelName].Equation(name='A66-3-c37', terms=((1.0, 'c3', 3), (-1.0, 'c7', 3)))


                    ##############-----X-direction--edge---##############
                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):

                        mdb.models[modelName].Equation(name='A66-2-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 2), (-1.0, 'btedge%s'%k, 2),(-1.0, 'RP4', 2)))
                        mdb.models[modelName].Equation(name='A66-2-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 2), (-1.0, 'bbedge%s'%j, 2),(-1.0, 'RP4', 2)))

                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                        mdb.models[modelName].Equation(name='A66-1-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 1), (-1.0, 'bledge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='A66-1-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 1), (-1.0, 'bredge%s'%j, 1)))
                        mdb.models[modelName].Equation(name='A66-2-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 2), (-1.0, 'bledge%s'%k, 2),(-1.0, 'RP4', 2)))
                        mdb.models[modelName].Equation(name='A66-2-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 2), (-1.0, 'bredge%s'%j, 2),(-1.0, 'RP4', 2)))
                        mdb.models[modelName].Equation(name='A66-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='A66-3-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 3), (-1.0, 'bredge%s'%j, 3)))
                    ##############-----Y-direction--edge---##############
                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                        mdb.models[modelName].Equation(name='A66-1-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 1), (-1.0, 'lbedge%s'%k, 1),(-1.0, 'RP6', 1)))
                        mdb.models[modelName].Equation(name='A66-1-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 1), (-1.0, 'rbedge%s'%j, 1),(-1.0, 'RP6', 1)))  
                        mdb.models[modelName].Equation(name='A66-2-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 2), (-1.0, 'lbedge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='A66-2-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 2), (-1.0, 'rbedge%s'%j, 2)))  
                        mdb.models[modelName].Equation(name='A66-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='A66-3-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 3), (-1.0, 'rbedge%s'%j, 3)))  

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='A66-1-ftedge-fbedge%s'%i, terms=((1.0, 'ftedge%s'%i, 1), (-1.0, 'fbedge%s'%l, 1),(-1.0, 'RP6', 1)))
                        mdb.models[modelName].Equation(name='A66-1-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 1), (-1.0, 'bbedge%s'%j, 1),(-1.0, 'RP6', 1)))
                        mdb.models[modelName].Equation(name='A66-3-ftedge-fbedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'fbedge%s'%l, 3)))
                        mdb.models[modelName].Equation(name='A66-3-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 3), (-1.0, 'bbedge%s'%j, 3)))


            if mode3==True and onlyPBC == False:
                    for i in mdb.models[modelName].loads.keys():
                            del mdb.models[modelName].loads[i]
                    for i in mdb.models[modelName].boundaryConditions.keys():
                            del mdb.models[modelName].boundaryConditions[i]            

                    ##############-----Set the relative displacement---##############
                    region = a.sets['RP4']
                    mdb.models[modelName].DisplacementBC(name='A66-1', createStepName='Step-1', 
                        region=region, u1=UNSET, u2=0.5*Dispx, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    region = a.sets['RP6']
                    mdb.models[modelName].DisplacementBC(name='A66-2', createStepName='Step-1', 
                        region=region, u1=0.5*Dispy, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)


                    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
                        createStepName='Step-1', variables=('NFORC', ))

                    mdb.Job(name='job-mode3', model= modelName, description='', type=ANALYSIS, 
                        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                        scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                    mdb.jobs['job-mode3'].submit(consistencyChecking=OFF)
                    mdb.jobs['job-mode3'].waitForCompletion()
                    o3 = session.openOdb(name='%s' % (path+'\job-mode3.odb'))
                 
                    odb = session.odbs['%s' % (path+'\job-mode3.odb')]

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                        NODAL, ((COMPONENT, 'RF2'), )), ), nodeSets=('RP4', ))

                    for i in session.xyDataObjects.keys():
                        RFx = session.xyDataObjects[i][0][1]


                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                        NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP6', ))

                    for i in session.xyDataObjects.keys():
                        RFy = session.xyDataObjects[i][0][1]


                    #####  forceA66 = math.sqrt(RFx*RFx+RFy*RFy)
                    forceA66= RFx
                    A66 = forceA66/H                               

                    ##############-----Caculate-for-A16---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_X+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("FRONTBC", ))

                    session.FreeBodyFromNodesElements(name='FreeBody-A16', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-A16', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceA16 = 0
                    for i in session.xyDataObjects.keys():
                        forceA16=forceA16+(session.xyDataObjects[i][0][1])
                    A16 = forceA16/L       


                    ##############-----Caculate-for-A26---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_Y+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("TOPBC", ))


                    session.FreeBodyFromNodesElements(name='FreeBody-A26', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-A26', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceA26 = 0
                    for i in session.xyDataObjects.keys():
                        forceA26=forceA26+(session.xyDataObjects[i][0][1])
                    A26 = forceA26/L       





            if A66==False or onlyPBC == True:
                    A66='N/A'
                    A16='N/A'
                    A26='N/A'


####********************************* Calculate the equivalent stiffness coefficient by deformation mode4 ***********************************************************************************************************

            if mode4==True:
                    for i in mdb.models[modelName].constraints.keys():
                            del mdb.models[modelName].constraints[i]

                    ##############-----X-direction--face---##############
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D11-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1),(-frontsxyz[i][2], 'RP4', 1)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D11-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D11-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3)))
                    ##############-----Y-direction--face---##############
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='D11-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='D11-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='D11-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3)))

                    ##############-----X-direction--vertex---##############
                    mdb.models[modelName].Equation(name='D11-1-c12', terms=((1.0, 'c1', 1), (-1.0, 'c2', 1),(-coc1[c1[0]][2], 'RP4', 1)))
                    mdb.models[modelName].Equation(name='D11-1-c43', terms=((1.0, 'c4', 1), (-1.0, 'c3', 1),(-coc4[c4[0]][2], 'RP4', 1)))
                    mdb.models[modelName].Equation(name='D11-1-c56', terms=((1.0, 'c5', 1), (-1.0, 'c6', 1),(-coc5[c5[0]][2], 'RP4', 1)))
                    mdb.models[modelName].Equation(name='D11-1-c87', terms=((1.0, 'c8', 1), (-1.0, 'c7', 1),(-coc8[c8[0]][2], 'RP4', 1)))
                    mdb.models[modelName].Equation(name='D11-2-c56', terms=((1.0, 'c5', 2), (-1.0, 'c6', 2)))
                    mdb.models[modelName].Equation(name='D11-2-c87', terms=((1.0, 'c8', 2), (-1.0, 'c7', 2)))
                    mdb.models[modelName].Equation(name='D11-3-c56', terms=((1.0, 'c5', 3), (-1.0, 'c6', 3)))
                    mdb.models[modelName].Equation(name='D11-3-c87', terms=((1.0, 'c8', 3), (-1.0, 'c7', 3)))

                    ##############-----Y-direction--vertex---##############
                    mdb.models[modelName].Equation(name='D11-2-c26', terms=((1.0, 'c2', 2), (-1.0, 'c6', 2)))
                    mdb.models[modelName].Equation(name='D11-3-c26', terms=((1.0, 'c2', 3), (-1.0, 'c6', 3)))
                    mdb.models[modelName].Equation(name='D11-2-c15', terms=((1.0, 'c1', 2), (-1.0, 'c5', 2)))
                    mdb.models[modelName].Equation(name='D11-3-c15', terms=((1.0, 'c1', 3), (-1.0, 'c5', 3)))
                    mdb.models[modelName].Equation(name='D11-2-c48', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2)))
                    mdb.models[modelName].Equation(name='D11-3-c48', terms=((1.0, 'c4', 3), (-1.0, 'c8', 3)))
                    mdb.models[modelName].Equation(name='D11-2-c37', terms=((1.0, 'c3', 2), (-1.0, 'c7', 2)))
                    mdb.models[modelName].Equation(name='D11-3-c37', terms=((1.0, 'c3', 3), (-1.0, 'c7', 3)))

                    ##############-----X-direction--edge---##############
                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='D11-1-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 1), (-1.0, 'btedge%s'%k, 1),(-ftedgexyz[i][2], 'RP4', 1)))
                        mdb.models[modelName].Equation(name='D11-1-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 1), (-1.0, 'bbedge%s'%j, 1),(-fbedgexyz[l][2], 'RP4', 1)))
                        mdb.models[modelName].Equation(name='D11-2-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 2), (-1.0, 'btedge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='D11-2-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 2), (-1.0, 'bbedge%s'%j, 2)))
                        mdb.models[modelName].Equation(name='D11-3-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'btedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='D11-3-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 3), (-1.0, 'bbedge%s'%j, 3)))

                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                        mdb.models[modelName].Equation(name='D11-1-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 1), (-1.0, 'bledge%s'%k, 1),(-fledgexyz[i][2], 'RP4', 1)))
                        mdb.models[modelName].Equation(name='D11-1-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 1), (-1.0, 'bredge%s'%j, 1),(-fredgexyz[l][2], 'RP4', 1)))
                        mdb.models[modelName].Equation(name='D11-2-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 2), (-1.0, 'bledge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='D11-2-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 2), (-1.0, 'bredge%s'%j, 2)))
                        mdb.models[modelName].Equation(name='D11-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='D11-3-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 3), (-1.0, 'bredge%s'%j, 3)))

                    ##############-----Y-direction--edge---##############
                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                        mdb.models[modelName].Equation(name='D11-1-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 1), (-1.0, 'lbedge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='D11-1-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 1), (-1.0, 'rbedge%s'%j, 1)))  
                        mdb.models[modelName].Equation(name='D11-2-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 2), (-1.0, 'lbedge%s'%k, 2),(-ltedgexyz[i][2], 'RP6', 2)))
                        mdb.models[modelName].Equation(name='D11-2-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 2), (-1.0, 'rbedge%s'%j, 2),(-rtedgexyz[l][2], 'RP6', 2)))  
                        mdb.models[modelName].Equation(name='D11-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='D11-3-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 3), (-1.0, 'rbedge%s'%j, 3)))  

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='D11-2-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 2), (-1.0, 'bbedge%s'%j, 2)))
                        mdb.models[modelName].Equation(name='D11-3-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 3), (-1.0, 'bbedge%s'%j, 3)))


            if mode4==True and onlyPBC == False:
                    for i in mdb.models[modelName].loads.keys():
                            del mdb.models[modelName].loads[i]
                    for i in mdb.models[modelName].boundaryConditions.keys():
                            del mdb.models[modelName].boundaryConditions[i]            

                    region = a.sets['RP4']
                    mdb.models[modelName].DisplacementBC(name='D11-1', createStepName='Step-1', 
                        region=region, u1=Dispx, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    region = a.sets['RP6']
                    mdb.models[modelName].DisplacementBC(name='D11-2', createStepName='Step-1', 
                        region=region, u1=UNSET, u2=0, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
                        createStepName='Step-1', variables=('NFORC', ))


                    mdb.Job(name='job-mode4', model= modelName, description='', type=ANALYSIS, 
                        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                        scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                    mdb.jobs['job-mode4'].submit(consistencyChecking=OFF)
                    mdb.jobs['job-mode4'].waitForCompletion()
                    o3 = session.openOdb(name='%s' % (path+'\job-mode4.odb'))
                    import os, glob                 
                    odb = session.odbs['%s' % (path+'\job-mode4.odb')]

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                        NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP4', ))

                    forceD11 = 0
                    for i in session.xyDataObjects.keys():
                        forceD11=forceD11+(session.xyDataObjects[i][0][1])

                    D11 = forceD11/H       

                    ##############-----Caculate-for-D12---##############

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name
                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=((
                        'FREEBODY', NODAL, ((COMPONENT, 'FB2'), )), ), nodeSets=("TOPBC", ))

                    NL_TOP = []
                    NL_TOP = topbc
                    NL_TOP.sort()  # the nodelabel of top node in ascending order

                    ii = 0
                    forceD12 = 0

                    for XYdata_name in session.xyDataObjects.keys():
                        nodelabel = NL_TOP[ii]
                        z3_coord = topbcxyz[nodelabel][2]
                        FBdata = session.xyDataObjects[XYdata_name][0][1]
                        forceD12 = forceD12 +z3_coord*FBdata
                        ii = ii+1

                    D12 = forceD12/L       


                    ##############-----Caculate-for-B11---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_X+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("FRONTBC", ))

                    session.FreeBodyFromNodesElements(name='FreeBody-2', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-2', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceB11 = 0
                    for i in session.xyDataObjects.keys():
                        forceB11=forceB11+(session.xyDataObjects[i][0][1])
                    B11 = forceB11/L       



            if D11==False or onlyPBC == True:
                    D11='N/A'
                    D12='N/A'
                    B11='N/A'


####********************************* Calculate the equivalent stiffness coefficient by deformation mode5 ***********************************************************************************************************
            if mode5==True:
                    for i in mdb.models[modelName].constraints.keys():
                            del mdb.models[modelName].constraints[i]


                    ##############-----Y-direction--face---##############
                    for i,k in zip(tops,bots):
                        if topsxyz[i][2] !=0 and botsxyz[k][2] !=0:
                            mdb.models[modelName].Equation(name='D22-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2),(-topsxyz[i][2], 'RP6', 2)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='D22-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='D22-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3)))
                    ##############-----X-direction--face---##############
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D22-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D22-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D22-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3)))

                    ##############-----X-direction--vertex---##############
                    mdb.models[modelName].Equation(name='D22-1-c12', terms=((1.0, 'c1', 1), (-1.0, 'c2', 1)))
                    mdb.models[modelName].Equation(name='D22-1-c43', terms=((1.0, 'c4', 1), (-1.0, 'c3', 1)))
                    mdb.models[modelName].Equation(name='D22-1-c56', terms=((1.0, 'c5', 1), (-1.0, 'c6', 1)))
                    mdb.models[modelName].Equation(name='D22-1-c87', terms=((1.0, 'c8', 1), (-1.0, 'c7', 1)))
                    mdb.models[modelName].Equation(name='D22-3-c12', terms=((1.0, 'c1', 3), (-1.0, 'c2', 3)))
                    mdb.models[modelName].Equation(name='D22-3-c43', terms=((1.0, 'c4', 3), (-1.0, 'c3', 3)))
                    mdb.models[modelName].Equation(name='D22-3-c56', terms=((1.0, 'c5', 3), (-1.0, 'c6', 3)))
                    mdb.models[modelName].Equation(name='D22-3-c87', terms=((1.0, 'c8', 3), (-1.0, 'c7', 3)))
                    ##############-----Y-direction--vertex---##############
                    mdb.models[modelName].Equation(name='D22-2-c26', terms=((1.0, 'c2', 2), (-1.0, 'c6', 2),(-coc2[c2[0]][2], 'RP6', 2)))
                    mdb.models[modelName].Equation(name='D22-2-c15', terms=((1.0, 'c1', 2), (-1.0, 'c5', 2),(-coc1[c1[0]][2], 'RP6', 2)))
                    mdb.models[modelName].Equation(name='D22-2-c48', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2),(-coc4[c4[0]][2], 'RP6', 2)))
                    mdb.models[modelName].Equation(name='D22-2-c37', terms=((1.0, 'c3', 2), (-1.0, 'c7', 2),(-coc3[c3[0]][2], 'RP6', 2)))

                    ##############-----X-direction--edge---##############
                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='D22-1-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 1), (-1.0, 'btedge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='D22-3-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'btedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='D22-1-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 1), (-1.0, 'bbedge%s'%j, 1)))
                        mdb.models[modelName].Equation(name='D22-3-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 3), (-1.0, 'bbedge%s'%j, 3)))

                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                        mdb.models[modelName].Equation(name='D22-1-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 1), (-1.0, 'bledge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='D22-1-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 1), (-1.0, 'bredge%s'%j, 1)))
                        mdb.models[modelName].Equation(name='D22-2-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 2), (-1.0, 'bledge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='D22-2-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 2), (-1.0, 'bredge%s'%j, 2)))
                        mdb.models[modelName].Equation(name='D22-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='D22-3-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 3), (-1.0, 'bredge%s'%j, 3)))

                    ##############-----Y-direction--edge---##############
                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                        mdb.models[modelName].Equation(name='D22-1-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 1), (-1.0, 'lbedge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='D22-1-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 1), (-1.0, 'rbedge%s'%j, 1)))  
                        mdb.models[modelName].Equation(name='D22-2-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 2), (-1.0, 'lbedge%s'%k, 2),(-ltedgexyz[i][2], 'RP6', 2)))
                        mdb.models[modelName].Equation(name='D22-2-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 2), (-1.0, 'rbedge%s'%j, 2),(-rtedgexyz[l][2], 'RP6', 2)))  
                        mdb.models[modelName].Equation(name='D22-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='D22-3-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 3), (-1.0, 'rbedge%s'%j, 3)))  

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='D22-2-ftedge-fbedge%s'%i, terms=((1.0, 'ftedge%s'%i, 2), (-1.0, 'fbedge%s'%l, 2),(-ftedgexyz[i][2], 'RP6', 2)))
                        mdb.models[modelName].Equation(name='D22-2-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 2), (-1.0, 'bbedge%s'%j, 2),(-btedgexyz[k][2], 'RP6', 2)))


            if mode5==True and onlyPBC == False:
                    for i in mdb.models[modelName].loads.keys():
                            del mdb.models[modelName].loads[i]
                    for i in mdb.models[modelName].boundaryConditions.keys():
                            del mdb.models[modelName].boundaryConditions[i]            

                    region = a.sets['RP6']
                    mdb.models[modelName].DisplacementBC(name='D22-1', createStepName='Step-1', 
                        region=region, u1=UNSET, u2=Dispy, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
                        createStepName='Step-1', variables=('NFORC', ))

                    mdb.Job(name='job-mode5', model= modelName, description='', type=ANALYSIS, 
                        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                        scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                    mdb.jobs['job-mode5'].submit(consistencyChecking=OFF)
                    mdb.jobs['job-mode5'].waitForCompletion()
                    o3 = session.openOdb(name='%s' % (path+'\job-mode5.odb'))
                    import os, glob
                 
                    odb = session.odbs['%s' % (path+'\job-mode5.odb')]

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                        NODAL, ((COMPONENT, 'RF2'), )), ), nodeSets=('RP6', ))

                    forceD22 = 0
                    for i in session.xyDataObjects.keys():
                        forceD22=forceD22+(session.xyDataObjects[i][0][1])

                    D22 = forceD22/L                             

                    ##############-----Caculate-for-B22---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_Y+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("TOPBC", ))


                    session.FreeBodyFromNodesElements(name='FreeBody-B22', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-B22', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceB22 = 0
                    for i in session.xyDataObjects.keys():
                        forceB22=forceB22+(session.xyDataObjects[i][0][1])
                    B22 = forceB22/L       

                    ##############-----Caculate-for-B21(12)---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_X+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("FRONTBC", ))

                    session.FreeBodyFromNodesElements(name='FreeBody-B12', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-B12', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceB12 = 0
                    for i in session.xyDataObjects.keys():
                        forceB12=forceB12+(session.xyDataObjects[i][0][1])
                    B12 = forceB12/L       


            if D22==False or onlyPBC == True:
                    D22='N/A'
                    B22='N/A'
                    B12='N/A'
                    B23='N/A'



####********************************* Calculate the equivalent stiffness coefficient by deformation mode6 ***********************************************************************************************************
            if mode6==True:
                    for i in mdb.models[modelName].constraints.keys():
                            del mdb.models[modelName].constraints[i]

                    ##############-----X-direction--face---##############
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D66-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D66-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2),(-frontsxyz[i][2], 'RP4', 2)))
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='D66-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3),(frontsxyz[i][1], 'RP4', 3)))
                    ##############-----Y-direction--face---##############
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='D66-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1),(-topsxyz[i][2], 'RP6', 1)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='D66-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2)))
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='D66-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3),(topsxyz[i][0], 'RP6', 3)))

                       #######-----1-direction--vertex---########
                    mdb.models[modelName].Equation(name='D66-1-c25', terms=((1.0, 'c2', 1), (-1.0, 'c5', 1),(-coc2[c2[0]][2], 'RP6', 1)))
                    mdb.models[modelName].Equation(name='D66-1-c38', terms=((1.0, 'c3', 1), (-1.0, 'c8', 1),(-coc3[c3[0]][2], 'RP6', 1)))
                    mdb.models[modelName].Equation(name='D66-1-c15', terms=((1.0, 'c1', 1), (-1.0, 'c5', 1),(-coc1[c1[0]][2], 'RP6', 1)))
                    mdb.models[modelName].Equation(name='D66-1-c48', terms=((1.0, 'c4', 1),(-1.0, 'c8', 1), (-coc4[c4[0]][2], 'RP6', 1)))
                    mdb.models[modelName].Equation(name='D66-1-c56', terms=((1.0, 'c5', 1), (-1.0, 'c6', 1)))
                    mdb.models[modelName].Equation(name='D66-1-c87', terms=((1.0, 'c8', 1), (-1.0, 'c7', 1)))
                    #############-----2-direction--vertex---##############

                    mdb.models[modelName].Equation(name='D66-2-c25', terms=((1.0, 'c2', 2), (-1.0, 'c5', 2),(coc5[c5[0]][2], 'RP4', 2)))
                    mdb.models[modelName].Equation(name='D66-2-c38', terms=((1.0, 'c3', 2), (-1.0, 'c8', 2),(coc8[c8[0]][2], 'RP4', 2)))
                    mdb.models[modelName].Equation(name='D66-2-c15', terms=((1.0, 'c1', 2), (-1.0, 'c5', 2)))
                    mdb.models[modelName].Equation(name='D66-2-c48', terms=((1.0, 'c4', 2),(-1.0, 'c8', 2)))
                    mdb.models[modelName].Equation(name='D66-2-c56', terms=((1.0, 'c5', 2), (-1.0, 'c6', 2),(-coc5[c5[0]][2], 'RP4', 2)))
                    mdb.models[modelName].Equation(name='D66-2-c87', terms=((1.0, 'c8', 2), (-1.0, 'c7', 2),(-coc8[c8[0]][2], 'RP4', 2)))

                    #############-----3-direction--vertex---##############

                    mdb.models[modelName].Equation(name='D66-3-c25', terms=((1.0, 'c2', 3), (-1.0, 'c5', 3)))
                    mdb.models[modelName].Equation(name='D66-3-c38', terms=((1.0, 'c3', 3), (-1.0, 'c8', 3)))
                    mdb.models[modelName].Equation(name='D66-3-c15', terms=((1.0, 'c1', 3), (-1.0, 'c5', 3),(coc1[c1[0]][0], 'RP6', 3)))
                    mdb.models[modelName].Equation(name='D66-3-c48', terms=((1.0, 'c4', 3),(-1.0, 'c8', 3), (coc4[c4[0]][0], 'RP6', 3)))
                    mdb.models[modelName].Equation(name='D66-3-c56', terms=((1.0, 'c5', 3), (-1.0, 'c6', 3),(coc5[c5[0]][1], 'RP4', 3)))
                    mdb.models[modelName].Equation(name='D66-3-c87', terms=((1.0, 'c8', 3), (-1.0, 'c7', 3),(coc8[c8[0]][1], 'RP4', 3)))


                    ##############-----X-direction--edge---##############

                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                        mdb.models[modelName].Equation(name='D66-1-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 1), (-1.0, 'bledge%s'%k, 1)))
                        mdb.models[modelName].Equation(name='D66-1-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 1), (-1.0, 'bredge%s'%j, 1)))
                        mdb.models[modelName].Equation(name='D66-2-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 2), (-1.0, 'bledge%s'%k, 2),(-fledgexyz[i][2], 'RP4', 2)))
                        mdb.models[modelName].Equation(name='D66-2-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 2), (-1.0, 'bredge%s'%j, 2),(-fredgexyz[l][2], 'RP4', 2)))
                        mdb.models[modelName].Equation(name='D66-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3),(fledgexyz[i][1], 'RP4', 3)))
                        mdb.models[modelName].Equation(name='D66-3-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 3), (-1.0, 'bredge%s'%j, 3),(fredgexyz[l][1], 'RP4', 3)))
                    ##############-----Y-direction--edge---##############
                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                        mdb.models[modelName].Equation(name='D66-1-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 1), (-1.0, 'lbedge%s'%k, 1),(-ltedgexyz[i][2], 'RP6', 1)))
                        mdb.models[modelName].Equation(name='D66-1-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 1), (-1.0, 'rbedge%s'%j, 1),(-rtedgexyz[l][2], 'RP6', 1)))  
                        mdb.models[modelName].Equation(name='D66-2-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 2), (-1.0, 'lbedge%s'%k, 2)))
                        mdb.models[modelName].Equation(name='D66-2-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 2), (-1.0, 'rbedge%s'%j, 2))) 
                        mdb.models[modelName].Equation(name='D66-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3),(ltedgexyz[i][0], 'RP6', 3)))
                        mdb.models[modelName].Equation(name='D66-3-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 3), (-1.0, 'rbedge%s'%j, 3),(rtedgexyz[l][0], 'RP6', 3)))  

                    ##############-----1-direction--edge---##############
                    for i,k in zip(btedge,fbedge):
                        mdb.models[modelName].Equation(name='D66-1-btedge-fbedge%s'%i, terms=((1.0, 'btedge%s'%i, 1), (-1.0, 'fbedge%s'%k, 1),(-btedgexyz[i][2], 'RP6', 1)))

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='D66-1-ftedge-fbedge%s'%i, terms=((1.0, 'ftedge%s'%i, 1),(-1.0, 'fbedge%s'%l, 1),(-ftedgexyz[i][2], 'RP6', 1)))
                        mdb.models[modelName].Equation(name='D66-1-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 1), (-1.0, 'bbedge%s'%j, 1)))

                    ##############-----2-direction--edge---##############
                    for i,k in zip(btedge,fbedge):
                        mdb.models[modelName].Equation(name='D66-2-btedge-fbedge%s'%i, terms=((1.0, 'btedge%s'%i, 2), (-1.0, 'fbedge%s'%k, 2),(fbedgexyz[k][2], 'RP4', 2)))

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='D66-2-ftedge-fbedge%s'%i, terms=((1.0, 'ftedge%s'%i, 2),(-1.0, 'fbedge%s'%l, 2)))
                        mdb.models[modelName].Equation(name='D66-2-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 2), (-1.0, 'bbedge%s'%j, 2),(-fbedgexyz[l][2], 'RP4', 2)))

                    ##############-----3-direction--edge---##############
                    for i,k in zip(btedge,fbedge):
                        mdb.models[modelName].Equation(name='D66-3-btedge-fbedge%s'%i, terms=((1.0, 'btedge%s'%i, 3), (-1.0, 'fbedge%s'%k, 3)))

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                        mdb.models[modelName].Equation(name='D66-3-ftedge-fbedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3),(-1.0, 'fbedge%s'%l, 3), (ftedgexyz[i][0], 'RP6', 3)))
                        mdb.models[modelName].Equation(name='D66-3-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 3), (-1.0, 'bbedge%s'%j, 3),(fbedgexyz[l][1], 'RP4', 3)))


            if mode6==True and onlyPBC == False:
                    for i in mdb.models[modelName].loads.keys():
                            del mdb.models[modelName].loads[i]
                    for i in mdb.models[modelName].boundaryConditions.keys():
                            del mdb.models[modelName].boundaryConditions[i]            

                    ##############-----Set the relative displacement---##############
                    region = a.sets['RP4']
                    mdb.models[modelName].DisplacementBC(name='D66-1', createStepName='Step-1', 
                        region=region, u1=UNSET, u2=0.5*Dispx, u3=0.5*Dispx, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    region = a.sets['RP6']
                    mdb.models[modelName].DisplacementBC(name='D66-2', createStepName='Step-1', 
                        region=region, u1=0.5*Dispy, u2=UNSET, u3=0.5*Dispy, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
                        createStepName='Step-1', variables=('NFORC', ))

                    mdb.Job(name='job-mode6', model= modelName, description='', type=ANALYSIS, 
                        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                        scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                    mdb.jobs['job-mode6'].submit(consistencyChecking=OFF)
                    mdb.jobs['job-mode6'].waitForCompletion()

                    o3 = session.openOdb(name='job-mode6.odb')
                    odb = session.odbs['job-mode6.odb']
                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name


                    ##############-----Caculate-for-D66---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                    NODAL, ((COMPONENT, 'RF2'), )), ), nodeSets=('RP4', ))

                    for k,v in session.xyDataObjects.items():
                        G12 = v[0][1]

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                    NODAL, ((COMPONENT, 'RF3'), )), ), nodeSets=('RP4', ))

                    for k,v in session.xyDataObjects.items():
                        G13 = v[0][1]

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                    NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP6', ))

                    for k,v in session.xyDataObjects.items():
                        G21 = v[0][1]

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                    NODAL, ((COMPONENT, 'RF3'), )), ), nodeSets=('RP6', ))
                    for k,v in session.xyDataObjects.items():
                        G23 = v[0][1]

                    forceD66 = 0.5*(G12/H + G21/L + G23/W + G13/W)
                    D66 = forceD66  


                    ##############-----Caculate-for-B16---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_X+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("FRONTBC", ))

                    session.FreeBodyFromNodesElements(name='FreeBody-B16', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-B16', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceB16 = 0
                    for i in session.xyDataObjects.keys():
                        forceB16=forceB16+(session.xyDataObjects[i][0][1])
                    B16 = forceB16/L       


                    ##############-----Caculate-for-B26---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_Y+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("TOPBC", ))


                    session.FreeBodyFromNodesElements(name='FreeBody-B26', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-B26', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceB26 = 0
                    for i in session.xyDataObjects.keys():
                        forceB26=forceB26+(session.xyDataObjects[i][0][1])
                    B26 = forceB26/L       



                    ##############-----Caculate-for-B66---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_X+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("FRONTBC", ))

                    session.FreeBodyFromNodesElements(name='FreeBody-B66', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-B66', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=OFF, comp2=ON, comp3=OFF)

                    forceB66 = 0
                    for i in session.xyDataObjects.keys():
                        forceB66=forceB66+(session.xyDataObjects[i][0][1])
                    B66 = forceB66/L       


                    ##############-----Caculate-for-D16---##############

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name
                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=((
                        'FREEBODY', NODAL, ((COMPONENT, 'FB1'), )), ), nodeSets=("FRONTBC", ))

                    NL_FRONT = []
                    NL_FRONT = frontbc
                    NL_FRONT.sort()  # the nodelabel of top node in ascending order

                    ii = 0
                    forceD16 = 0

                    for XYdata_name in session.xyDataObjects.keys():
                        nodelabel = NL_FRONT[ii]
                        z3_coord = frontbcxyz[nodelabel][2]
                        FBdata = session.xyDataObjects[XYdata_name][0][1]
                        forceD16 = forceD16 +z3_coord*FBdata
                        ii = ii+1

                    D16 = forceD16/L       


                    ##############-----Caculate-for-D26---##############

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name
                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=((
                        'FREEBODY', NODAL, ((COMPONENT, 'FB2'), )), ), nodeSets=("TOPBC", ))

                    NL_TOP = []
                    NL_TOP = topbc
                    NL_TOP.sort()  # the nodelabel of top node in ascending order

                    ii = 0
                    forceD26 = 0

                    for XYdata_name in session.xyDataObjects.keys():
                        nodelabel = NL_TOP[ii]
                        z3_coord = topbcxyz[nodelabel][2]
                        FBdata = session.xyDataObjects[XYdata_name][0][1]
                        forceD26 = forceD26 +z3_coord*FBdata
                        ii = ii+1

                    D26 = forceD26/L       



            if mode6==False or onlyPBC == True:
                    D66='N/A'
                    B16='N/A'
                    B26='N/A'
                    B66='N/A'
                    D16='N/A'
                    D26='N/A'



####********************************* Calculate the equivalent stiffness coefficient by deformation mode7 ***********************************************************************************************************
            if mode7==True :
                    if onlyPBC == False:
                            for i in mdb.models[modelName].constraints.keys():
                                    del mdb.models[modelName].constraints[i]

                    ##############-----X-direction--face---##############
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='H11-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1)))

                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='H11-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3)))

                    ##############-----Y-direction--face---##############
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='H11-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1)))

                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='H11-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3),(-1.0, 'RP6', 3)))

                    ##############-----X-direction--vertex---##############
                    mdb.models[modelName].Equation(name='H11-1-c12', terms=((1.0, 'c1', 1), (-1.0, 'c2', 1)))
                    mdb.models[modelName].Equation(name='H11-1-c43', terms=((1.0, 'c4', 1), (-1.0, 'c3', 1)))
                    mdb.models[modelName].Equation(name='H11-1-c56', terms=((1.0, 'c5', 1), (-1.0, 'c6', 1)))
                    mdb.models[modelName].Equation(name='H11-1-c87', terms=((1.0, 'c8', 1), (-1.0, 'c7', 1)))

                    ##############-----Y-direction--vertex---##############
                    mdb.models[modelName].Equation(name='H11-3-c26', terms=((1.0, 'c2', 3), (-1.0, 'c6', 3), (-1.0, 'RP6', 3)))
                    mdb.models[modelName].Equation(name='H11-3-c15', terms=((1.0, 'c1', 3), (-1.0, 'c5', 3), (-1.0, 'RP6', 3)))
                    mdb.models[modelName].Equation(name='H11-3-c48', terms=((1.0, 'c4', 3), (-1.0, 'c8', 3), (-1.0, 'RP6', 3)))
                    mdb.models[modelName].Equation(name='H11-3-c37', terms=((1.0, 'c3', 3), (-1.0, 'c7', 3), (-1.0, 'RP6', 3)))



                    ##############-----X-direction--edge---##############
                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):

                        mdb.models[modelName].Equation(name='H11-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='H11-3-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 3), (-1.0, 'bredge%s'%j, 3)))

                    ##############-----Y-direction--edge---##############
                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):

                        mdb.models[modelName].Equation(name='H11-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3),(-1.0, 'RP6', 3)))
                        mdb.models[modelName].Equation(name='H11-3-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 3), (-1.0, 'rbedge%s'%j, 3),(-1.0, 'RP6', 3)))  

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):

                        mdb.models[modelName].Equation(name='H11-3-ftedge-fbedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'fbedge%s'%l, 3),(-1.0, 'RP6', 3)))
                        mdb.models[modelName].Equation(name='H11-3-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 3), (-1.0, 'bbedge%s'%j, 3),(-1.0, 'RP6', 3)))

                    ss = []
                    filename = mass_file
                    f = open(filename,"r")   #设置文件对象
                    s = f.read().splitlines()
                    for i in s:
                        i = float(i)
                        ss.append(i)

                    cubeterm = []
                    lump = []
                    term = []
                    for i in cubexyz.keys():
                        mass= ss[i-1]
                        z = cubexyz[i][2]
                        lump = mass*z
                        terms=(lump,'cube%s'%i,2 )
                        cubeterm.append(terms)


                    mdb.models[modelName].Equation(name='H11-2-3', terms=(cubeterm))


            if mode7==True and onlyPBC == False:
                    for i in mdb.models[modelName].loads.keys():
                            del mdb.models[modelName].loads[i]
                    for i in mdb.models[modelName].boundaryConditions.keys():
                            del mdb.models[modelName].boundaryConditions[i]

                    region = a.sets['RP6']
                    mdb.models[modelName].DisplacementBC(name='H11-1', createStepName='Step-1', 
                        region=region, u1=UNSET, u2=UNSET, u3=Dispy, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
                        createStepName='Step-1', variables=('NFORC', ))

                    import os, glob

                    mdb.Job(name='job-mode7', model= modelName, description='', type=ANALYSIS, 
                        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                        scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                    mdb.jobs['job-mode7'].submit(consistencyChecking=OFF)

                    mdb.jobs['job-mode7'].waitForCompletion()


                    o3 = session.openOdb(name='%s' % (path+'\job-mode7.odb'))


                    odb = session.odbs['%s' % (path+'\job-mode7.odb')]

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name

                    ##############-----Caculate-for-H11---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                        NODAL, ((COMPONENT, 'RF3'), )), ), nodeSets=('RP6', ))

                    forceH11 = 0
                    for i in session.xyDataObjects.keys():
                        forceH11=forceH11+(session.xyDataObjects[i][0][1])

                    H11 = forceH11/L

                    ##############-----Caculate-for-K11---##############

                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    eLeaf = dgo.LeafFromElementSets(elementSets=("FACE_X+", ))
                    nLeaf = dgo.LeafFromNodeSets(nodeSets=("FRONTBC", ))

                    session.FreeBodyFromNodesElements(name='FreeBody-K11', elements=eLeaf, 
                        nodes=nLeaf, summationLoc=CENTROID, componentResolution=NORMAL_TANGENTIAL)
                    session.viewports['Viewport: 1'].odbDisplay.setValues(freeBodyNames=(
                        'FreeBody-K11', ), freeBody=ON)

                    session.XYDataFromFreeBody(odb=odb, force=ON, moment=OFF, heatFlowRate=OFF, 
                        resultant=OFF, comp1=ON, comp2=OFF, comp3=OFF)

                    forceK11 = 0
                    for i in session.xyDataObjects.keys():
                        forceK11=forceK11+(session.xyDataObjects[i][0][1])
                    K11 = forceK11/L       



            if mode7==False or onlyPBC == True:
                    H11='N/A'
                    K11='N/A'
####********************************* Calculate the equivalent stiffness coefficient by deformation mode8 ***********************************************************************************************************


            if mode8==True :
                    if onlyPBC == False:
                            for i in mdb.models[modelName].constraints.keys():
                                    del mdb.models[modelName].constraints[i]


                    ##############-----X-direction--face---##############
                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='H22-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2)))

                    for i,k in zip(fronts,backs):
                        mdb.models[modelName].Equation(name='H22-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3),(-1.0, 'RP4', 3)))

                    ##############-----Y-direction--face---##############
                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='H22-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2)))

                    for i,k in zip(tops,bots):
                        mdb.models[modelName].Equation(name='H22-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3)))

                    ##############-----X-direction--vertex---##############
                    mdb.models[modelName].Equation(name='H22-3-c12', terms=((1.0, 'c1', 3), (-1.0, 'c2', 3), (-1.0, 'RP4', 3)))
                    mdb.models[modelName].Equation(name='H22-3-c43', terms=((1.0, 'c4', 3), (-1.0, 'c3', 3), (-1.0, 'RP4', 3)))
                    mdb.models[modelName].Equation(name='H22-3-c56', terms=((1.0, 'c5', 3), (-1.0, 'c6', 3), (-1.0, 'RP4', 3)))
                    mdb.models[modelName].Equation(name='H22-3-c87', terms=((1.0, 'c8', 3), (-1.0, 'c7', 3), (-1.0, 'RP4', 3)))

                    ##############-----Y-direction--vertex---##############
                    mdb.models[modelName].Equation(name='H22-2-c26', terms=((1.0, 'c2', 2), (-1.0, 'c6', 2)))
                    mdb.models[modelName].Equation(name='H22-2-c15', terms=((1.0, 'c1', 2), (-1.0, 'c5', 2)))
                    mdb.models[modelName].Equation(name='H22-2-c48', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2)))
                    mdb.models[modelName].Equation(name='H22-2-c37', terms=((1.0, 'c3', 2), (-1.0, 'c7', 2)))


                    for i,k,j,l in zip(fledge,bledge,bredge,fredge):

                        mdb.models[modelName].Equation(name='H22-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3),(-1.0, 'RP4', 3)))
                        mdb.models[modelName].Equation(name='H22-3-fredge-bredge%s'%l, terms=((1.0, 'fredge%s'%l, 3), (-1.0, 'bredge%s'%j, 3),(-1.0, 'RP4', 3)))


                    for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):

                        mdb.models[modelName].Equation(name='H22-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3)))
                        mdb.models[modelName].Equation(name='H22-3-rtedge-rbedge%s'%l, terms=((1.0, 'rtedge%s'%l, 3), (-1.0, 'rbedge%s'%j, 3)))  

                    for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):

                        mdb.models[modelName].Equation(name='H22-3-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'btedge%s'%k, 3),(-1.0, 'RP4', 3)))
                        mdb.models[modelName].Equation(name='H22-3-fbedge-bbedge%s'%l, terms=((1.0, 'fbedge%s'%l, 3), (-1.0, 'bbedge%s'%j, 3),(-1.0, 'RP4', 3)))


                    ss = []
                    filename = mass_file
                    f = open(filename,"r")   #设置文件对象
                    s = f.read().splitlines()
                    for i in s:
                        i = float(i)
                        ss.append(i)
                    cubeterm = []
                    lump = []
                    term = []
                    for i in cubexyz.keys():
                        mass= ss[i-1]
                        z = cubexyz[i][2]
                        lump = mass*z
                        terms=(lump,'cube%s'%i,1 )
                        cubeterm.append(terms)

                    mdb.models[modelName].Equation(name='H22-1-3', terms=(cubeterm))


            if mode8==True and onlyPBC == False:
                    for i in mdb.models[modelName].loads.keys():
                            del mdb.models[modelName].loads[i]
                    for i in mdb.models[modelName].boundaryConditions.keys():
                            del mdb.models[modelName].boundaryConditions[i]

                    region = a.sets['RP4']
                    mdb.models[modelName].DisplacementBC(name='H22-1', createStepName='Step-1', 
                        region=region, u1=UNSET, u2=UNSET, u3=Dispx, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                        localCsys=None)

                    import os, glob

                    mdb.Job(name='job-mode8', model= modelName, description='', type=ANALYSIS, 
                        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                        scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                    mdb.jobs['job-mode8'].submit(consistencyChecking=OFF)

                    mdb.jobs['job-mode8'].waitForCompletion()


                    o3 = session.openOdb(name='%s' % (path+'\job-mode8.odb'))


                    odb = session.odbs['%s' % (path+'\job-mode8.odb')]

                    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                    odbName=session.viewports[session.currentViewportName].odbDisplay.name


                    for i in session.xyDataObjects.keys():
                        del session.xyDataObjects['%s' % (i)]

                    session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                    session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                        NODAL, ((COMPONENT, 'RF3'), )), ), nodeSets=('RP4', ))

                    forceH22 = 0
                    for i in session.xyDataObjects.keys():
                        forceH22=forceH22+(session.xyDataObjects[i][0][1])

                    H22 = forceH22/H
                    
            if mode8==False or onlyPBC == True:
                    H22='N/A'


############### OUTPUT equivalent data of calculation  ###################################################################################################################################################

            density = 0
            if mass != None:
                    density = mass/(L*W*H)
                    surface_density = mass/(L*H)
            
            print ('----------------------------------------------------')
            print ('----------------------------------------------------')
            print ('The homogenised stiffness coefficients to the periodic model')
            print ('A11=%s Stress Unit of force/length unit' % (A11))
            print ('A22=%s Stress Unit of force/length unit' % (A22))
            print ('A12=%s Stress Unit of force/length unit' % (A12))
            print ('A66=%s Stress Unit of force/length unit' % (A66))
            print ('A16=%s Stress Unit of force/length unit' % (A16))
            print ('A26=%s Stress Unit of force/length unit' % (A26))
            print ('B11=%s Stress Unit of force' % (B11))
            print ('B22=%s Stress Unit of force' % (B22))
            print ('B12=%s Stress Unit of force' % (B12))
            print ('B66=%s Stress Unit of force' % (B66))
            print ('B16=%s Stress Unit of force' % (B16))
            print ('B26=%s Stress Unit of force' % (B26))
            print ('D11=%s Stress Unit of force*length unit' % (D11))
            print ('D22=%s Stress Unit of force*length unit' % (D22))
            print ('D12=%s Stress Unit of force*length unit' % (D12))
            print ('D66=%s Stress Unit of force*length unit' % (D66))
            print ('D16=%s Stress Unit of force*length unit' % (D16))
            print ('D26=%s Stress Unit of force*length unit' % (D26))
            print ('H11=%s Stress Unit of force/length unit' % (H11))
            print ('H22=%s Stress Unit of force/length unit' % (H22))

            print ('----------------------------------------------------')
            print ('Total mass=%s Mass units' % (mass))
            print ('Homogenised density=%s Density units' % (density))
#            print ('Homogenised surface density=%s surface Density units' % (surface_density))
            print ('----------------------------------------------------')
            print ('Processing duration %s seconds' % (time.time()-start))
            print ('----------------------------------------------------')
            
            filename = ('%s_equivalent_stiffness_coefficients.txt' % part)
            print ('The homogenised elastic properties are saved in ABAQUS Work Directory under %s' % filename)
            f = open(filename,'w')
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('Property','Value','Unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('A11',A11,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('A22',A22,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('A12',A12,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('A66',A66,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('A16',A16,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('A26',A26,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('B11',B11,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('B22',B22,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('B12',B12,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('B16',B16,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('B26',B26,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('B66',B66,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('D11',D11,'Unit of force*length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('D22',D22,'Unit of force*length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('D12',D12,'Unit of force*length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('D66',D66,'Unit of force*length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('D16',D16,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('D26',D26,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('H11',H11,'Unit of force/length unit'))
            f.write('{0:^10}{1:^20}{2:^20}\n'.format('H22',H22,'Unit of force/length unit'))


            f.write ('Total mass=%s Mass units \n' % (mass))
            f.write ('Homogenised density=%s Density units \n' % (density))
#            f.write('Homogenised surface density=%s surface Density units' % (surface_density))
            f.write ('processing duration %s Seconds' % (time.time()-start))

            f.close()


            filename = ('%s_equivalent_stiffness_coefficients(easycopy).txt' % part)
            f = open(filename,'w')

            f.write('{0:^10}\n'.format(A11))
            f.write('{0:^10}\n'.format(A22))
            f.write('{0:^10}\n'.format(A12))
            f.write('{0:^10}\n'.format(A66))
            f.write('{0:^10}\n'.format(A16))
            f.write('{0:^10}\n'.format(A26))
            f.write('{0:^10}\n'.format(B11))
            f.write('{0:^10}\n'.format(B22))
            f.write('{0:^10}\n'.format(B12))
            f.write('{0:^10}\n'.format(B16))
            f.write('{0:^10}\n'.format(B26))
            f.write('{0:^10}\n'.format(B66))
            f.write('{0:^10}\n'.format(D11))
            f.write('{0:^10}\n'.format(D22))
            f.write('{0:^10}\n'.format(D12))
            f.write('{0:^10}\n'.format(D66))
            f.write('{0:^10}\n'.format(D16))
            f.write('{0:^10}\n'.format(D26))
            f.write('{0:^10}\n'.format(H11))
            f.write('{0:^10}\n'.format(H22))
            f.write ('{0:^10}\n' .format(mass))
            f.write ('{0:^10}\n' .format(density)) 
#            f.write ('{0:^10}\n' .format(surface_density)) 
            f.write ('{0:^10}\n' .format((time.time()-start)))

            f.close()



            print ('----------------------------------------------------')
            if onlyPBC == True:
                    print ('EasyNPT created Period Boundary Conditions only. For further investigation, used relevant Reference Points to apply loads/displacements based on your needs. Details on the use of Reference Points are illustrated in Table 1 of the referred paper.')

            for i in session.xyDataObjects.keys():
                del session.xyDataObjects['%s' % (i)]
            print ('---------------------------------------')
            print ('--------- End of EasyNPT (3D) ---------')
            print ('---------------------------------------')

            if len(session.odbData.keys()) >= 1:                              
                    odb.close(odb, write=TRUE)
                    a = mdb.models[modelName].rootAssembly
                    session.viewports['Viewport: 1'].setValues(displayedObject=a)

        if error==True:
            print ('Error(s) found during sets creation, please check the error No.(s) above with EasyPBC user guide.')
            
            a.SetFromNodeLabels(name='Error set', nodeLabels=((instanceName,errorset),))


