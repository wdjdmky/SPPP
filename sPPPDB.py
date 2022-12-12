# Do not edit this file or it may not load correctly
# if you try to open it with the RSG Dialog Builder.

# Note: thisDir is defined by the Activator class when
#       this file gets executed

from rsg.rsgGui import *
from abaqusConstants import INTEGER, FLOAT
execDir = os.path.split(thisDir)[1]
dialogBox = RsgDialog(title='Stiffness Prediction for Periodic Plate (SPPP)', kernelModule='SPPP', kernelFunction='npt', includeApplyBtn=True, includeSeparator=True, okBtnText='OK', applyBtnText='Apply', execDir=execDir)
RsgGroupBox(name='GroupBox_1', p='DialogBox', text='Model and analysis details', layout='0')
RsgTextField(p='GroupBox_1', fieldType='String', ncols=12, labelText='Model name', keyword='part', default='Model-1')
RsgTextField(p='GroupBox_1', fieldType='String', ncols=12, labelText='Instance name', keyword='inst', default='Part-1-1')
RsgTextField(p='GroupBox_1', fieldType='String', ncols=12, labelText='Mapping accuracy', keyword='meshsens', default='10E-3')
RsgTextField(p='GroupBox_1', fieldType='String', ncols=12, labelText='Number of CPUs to be used **', keyword='CPU', default='1')
RsgLabel(p='GroupBox_1', text='*  Mesh mapping accuracy of opposite sides should be within the above value', useBoldFont=False)
RsgLabel(p='GroupBox_1', text='** If number of CPUs exceeds the available, all available CPUs will be used', useBoldFont=False)
RsgGroupBox(name='GroupBox_2', p='GroupBox_1', text='Stiffness Prediction for Periodic Plate', layout='0')
RsgCheckButton(p='GroupBox_2', text='Mode1', keyword='mode1', default=False)
RsgLabel(p='GroupBox_2', text='       +Tensile deformation in X diretion', useBoldFont=False)
RsgCheckButton(p='GroupBox_2', text='Mode2', keyword='mode2', default=False)
RsgLabel(p='GroupBox_2', text='       +Tensile deformation in Y diretion', useBoldFont=False)
RsgCheckButton(p='GroupBox_2', text='Mode3', keyword='mode3', default=False)
RsgLabel(p='GroupBox_2', text='       +Shear deformation in-plane', useBoldFont=False)
RsgCheckButton(p='GroupBox_2', text='Mode4', keyword='mode4', default=False)
RsgLabel(p='GroupBox_2', text='       +Bending deformation about X axis', useBoldFont=False)
RsgCheckButton(p='GroupBox_2', text='Mode5', keyword='mode5', default=False)
RsgLabel(p='GroupBox_2', text='       +Bending deformation about Y axis', useBoldFont=False)
RsgCheckButton(p='GroupBox_2', text='Mode6', keyword='mode6', default=False)
RsgLabel(p='GroupBox_2', text='       +Torsional deformation', useBoldFont=False)
RsgCheckButton(p='GroupBox_2', text='Mode7', keyword='mode7', default=False)
RsgLabel(p='GroupBox_2', text='       +Shear deformation out-of-plane in X direction', useBoldFont=False)
RsgCheckButton(p='GroupBox_2', text='Mode8', keyword='mode8', default=False)
RsgLabel(p='GroupBox_2', text='       +Shear deformation out-of-plane in Y direction', useBoldFont=False)
RsgCheckButton(p='GroupBox_2', text='If ticked, only BCconstrait BC will be created, stiffness will not be estimated', keyword='onlyPBC', default=False)
dialogBox.show()