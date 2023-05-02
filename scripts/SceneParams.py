# HOW TO:
#
# 1. Choose the patient directory.
#
#    The directory should contain:
#    a. meshes of the prostate, bladder, rectum, and lesions
#    See data/Patient4 and data/Patient6 for examples
#
# 2. Modify mechanical parameters in this file if necessary.
# 3. Modify the position of tissues if necessary.
# 4. Set initial configuration of the robot


# Step 1 : Here you set the path to the patient directory
import os

patient = 'data/Patient6/'
pathPatient = os.getcwd() + '/../' + patient
if not os.path.isdir(pathPatient):
    pathPatient = os.getcwd() + '/' + patient

# Computes the corresponding transformations and scene parameters
from importlib.machinery import SourceFileLoader

mriparams = SourceFileLoader('mriparams', pathPatient + 'params.py').load_module()
planeMeshSize = [mriparams.transform[6] * mriparams.MRISize[0], mriparams.transform[7] * mriparams.MRISize[1]]
gridMin = [-mriparams.transform[1], -mriparams.transform[0], mriparams.transform[2]]
gridMax = [gridMin[0] + mriparams.transform[6] * mriparams.MRISize[0],
           gridMin[1] + mriparams.transform[7] * mriparams.MRISize[1],
           gridMin[2] + mriparams.transform[8] * mriparams.MRISize[2]]

# Sets the paths to the meshes
prostatePath = pathPatient + 'prostateSimplified.obj'
prostateCapsulePath = pathPatient + 'prostateCapsuleSimplified.obj'
prostateColliPath = pathPatient + 'prostateSimplifiedCollision.obj'
bladderPath = pathPatient + 'bladderSimplified.obj'
rectumPath = pathPatient + 'rectumSimplified.obj'
rectumColliPath = pathPatient + 'rectumSimplifiedCollision.obj'
bonesPath = pathPatient + 'bones.obj'
lesionsPath = [pathPatient + 'sphere1.stl', pathPatient + 'sphere2.stl', pathPatient + 'sphere3.stl',
               pathPatient + 'sphere4.stl']

# Step 2 : Here you can change the mechanical parameters
youngModulusTissueGrid = 5
poissonRatioTissueGrid = 0
youngModulusProstate = 1
poissonRatioProstate = 0
youngModulusRobot = 18000
poissonRatioRobot = 0.45

# Step 3 : Here you can move the tissues in the scene
rotation = [0, 0, 0]
translation = [0, 0, 0]

# Step 4 : Initial configuration of the robot
# You can use the scene scripts/Robot.py to try a set of initial angles and compute
# the corresponding position of the robot effector and soft part extremity
# Once it is done, change the values here
robotBasePosition = [-180, -530, -300]
initAngles = [-0.05, -0.8, 0.07, 0., -0.75, -0.05]
initEffectorPosition = [174.801, 455.27, 11.3877]
initEffectorOrientation = [-0.699288, -0.0007461, -0.00087473, 0.71484]
initSoftExtremityPosition = [174.573, 463.068, 111.583]
initSoftExtremityOrientation = [-0.493944, -0.494999, -0.506087, 0.50485]
