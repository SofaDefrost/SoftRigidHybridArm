from importlib.machinery import SourceFileLoader
import os
from math import sin, cos, pi

import Sofa
import Sofa.Core
from scripts.RobotGUI import RobotGUI
from scripts.RobotSoftPart import addSoftPart
from splib3.numerics.quat import Quat

filename = os.getcwd() + '/scripts/SceneParams.py'
filename = filename.replace("/scripts/scripts", "/scripts")
params = SourceFileLoader('params', filename).load_module()

path = os.path.dirname(os.path.abspath(__file__)) + '/../data/Robot/Parts/'
basePath = path + 'RobotBase.stl'
part11Path = path + 'RobotPart (1).stl'
part12Path = path + 'RobotPart (3).stl'
part21Path = path + 'RobotPart (2).stl'
part22Path = path + 'RobotPart (4).stl'
part3Path = path + 'RobotPart (6).stl'
part41Path = path + 'RobotPart (5).stl'
part42Path = path + 'RobotPart (9).stl'
part51Path = path + 'RobotPart (7).stl'
part52Path = path + 'RobotPart (8).stl'
part6Path = path + 'RobotPart (10).stl'


# A trick to initialize the configuration of the robot
# Without that we would a problem between the robot and the soft part
class MappingController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.node = kwargs["node"]
        self.mapping = kwargs["mapping"]
        self.node.removeObject(self.mapping)
        self.once = True

    def onAnimateBeginEvent(self, e):
        if self.once:
            self.node.addObject(self.mapping)
            self.mapping.init()
            self.once = False


def addVisu(node, index, filename, translation=[0, 0, 0]):
    if filename is None:
        return

    visu = node.addChild('Visu' + str(index))
    visu.addObject('MeshSTLLoader', name='loader', filename=filename, translation=translation)
    visu.addObject('MeshTopology', src='@loader')
    visu.addObject('OglModel', color=[0.5, 0.5, 0.5, 1.])
    visu.addObject('RigidMapping')

    return


def addPart(node, name, index, filename1, filename2=None, translation=[0, 0, 0], rotation=[0, 0, 0]):
    part = node.addChild(name)
    part.addObject('MechanicalObject', template='Rigid3', position=[0, 0, 0, 0, 0, 0, 1])
    part.addObject('RigidMapping', index=index, globalToLocalCoords=True)

    addVisu(part, 1, filename1, translation=translation)
    addVisu(part, 2, filename2, translation=translation)

    return part


def addCenter(node, name,
              parentIndex, childIndex,
              posOnParent, posOnChild,
              articulationProcess,
              isTranslation, isRotation, axis,
              articulationIndex):
    center = node.addChild(name)
    center.addObject('ArticulationCenter', parentIndex=parentIndex, childIndex=childIndex, posOnParent=posOnParent,
                     posOnChild=posOnChild, articulationProcess=articulationProcess)

    articulation = center.addChild('Articulation')
    articulation.addObject('Articulation', translation=isTranslation, rotation=isRotation, rotationAxis=axis,
                           articulationIndex=articulationIndex)

    return center


def createRobot(node, name='Robot', translation=[0, 0, 0], initConfiguration=None,
                withSolver=True, inverse=False, withSoftPart=True, withEffector=True):
    # Positions of parts
    robotExtremityPosition = [0., 0., 0., 0., 0., sin(-pi / 4), cos(-pi / 4)]
    positions = [
        [160.8, 0, 160.8, 0, 0, 0, 1],
        [160.8, 78.5, 160.8, 0, 0, 0, 1],
        [254.8, 171, 160.8, 0, 0, 0, 1],
        [347.3, 372, 160.8, 0, 0, 0, 1],
        [254.8, 569.6, 160.8, 0, 0, 0, 1],
        [160.8, 500.5, 160.8, 0, 0, 0, 1],
        [160.8, 442.5, 160.8, 0, 0, 0, 1]
    ]

    initAngles = [0, 0, 0, 0, 0, 0]
    initEffectorPosition = None
    if initConfiguration is not None:
        if len(initConfiguration) == 2 and len(initConfiguration[0]) == 6 and len(initConfiguration[1]) == 7:
            initAngles = initConfiguration[0]
            initEffectorPosition = initConfiguration[1]
        else:
            Sofa.msg_error('[createRobot] initConfiguration should be [initAngles, initEffectorPosition]')
            initConfiguration = None

    # Robot node
    robot = node.addChild(name)
    if withSolver:
        robot.addObject('EulerImplicitSolver', rayleighMass=0.01)
        robot.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
        robot.addObject('GenericConstraintCorrection')

    robot.addData('angles', initAngles, None, 'angle of articulations in radian', '', 'vector<float>')
    robot.addData('packetOut', [0, 0, 0, 0, 0, 0], None, 'angle of articulations in degree', '',
                  'vector<int>')  # will be filled in RobotGUI.py
    for i in range(1, 7):
        robot.addData('minAngles' + str(i), -pi, None, 'min angle of part' + str(i), '', 'float')
        robot.addData('maxAngles' + str(i), pi, None, 'max angle of part' + str(i), '', 'float')

    # Articulations node
    articulations = robot.addChild('Articulations')
    articulations.addObject('MechanicalObject', name='dofs', template='Vec1',
                            rest_position=robot.getData('angles').getLinkPath(), position=initAngles)
    articulations.addObject('ArticulatedHierarchyContainer')
    articulations.addObject('UniformMass', totalMass=20)

    # Rigid
    rigid = articulations.addChild('Rigid')
    rigid.addObject('MechanicalObject', name='dofs', template='Rigid3', showObject=True, showObjectScale=10,
                    position=positions[0:7],
                    translation=translation)
    rigid.addObject('ArticulatedSystemMapping', input1=articulations.dofs.getLinkPath(),
                    output=rigid.dofs.getLinkPath())

    # Visu
    visu = rigid.addChild('Visu')
    addPart(visu, 'Base', 0, basePath, translation=translation)
    addPart(visu, 'Part1', 1, part11Path, part12Path, translation=translation)
    addPart(visu, 'Part2', 2, part21Path, part22Path, translation=translation)
    addPart(visu, 'Part3', 3, part3Path, translation=translation)
    addPart(visu, 'Part4', 4, part41Path, part42Path, translation=translation)
    addPart(visu, 'Part5', 5, part51Path, part52Path, translation=translation)
    addPart(visu, 'Part6', 6, part6Path, translation=translation)

    # Center of articulations
    centers = articulations.addChild('ArticulationsCenters')
    addCenter(centers, 'CenterBase', 0, 1, [0, 78.5, 0], [0, 0, 0], 0, 0, 1, [0, 1, 0], 0)
    addCenter(centers, 'CenterPart1', 1, 2, [94, 92.5, 0], [0, 0, 0], 0, 0, 1, [1, 0, 0], 1)
    addCenter(centers, 'CenterPart2', 2, 3, [92.5, 92.5, 0], [0, -108.5, 0], 0, 0, 1, [0, 1, 0], 2)
    addCenter(centers, 'CenterPart3', 3, 4, [0, 108.5, 0], [92.5, -89.1, 0], 0, 0, 1, [0, 0, 0], 3)
    addCenter(centers, 'CenterPart4', 4, 5, [0, 0, 0], [94, 69.1, 0], 0, 0, 1, [1, 0, 0], 4)
    addCenter(centers, 'CenterPart5', 5, 6, [0, 0, 0], [0, 58, 0], 0, 0, 1, [0, 1, 0], 5)

    if withSoftPart:
        # Soft part node
        softExtremity = robot.addChild('SoftExtremity')
        softExtremity.addObject('MechanicalObject', template='Rigid3', name='dofs', showObject=True, showObjectScale=20)

        robotExtremityPosition = rigid.dofs.position[
            6] if initEffectorPosition is None else initEffectorPosition  # Should always be rigid.dofs.position[6] but it is not properly initialized :(
        q = Quat(robotExtremityPosition[3:7])
        q.rotateFromQuat(Quat(0., 0., sin(-pi / 4), cos(-pi / 4)))
        robotExtremityPosition[3:7] = list(q)
        cylinder = addSoftPart(robot, rigid, softExtremity, robotExtremityPosition, translation, 'SoftCylinder')
        rigid.RigidifiedNodes1.RigidMapping.index = 6
        if initEffectorPosition is not None:
            # A trick to initialize the configuration of the robot
            # Without that we would a problem between the robot and the soft part
            node.addObject(MappingController(node=rigid.RigidifiedNodes1, mapping=rigid.RigidifiedNodes1.RigidMapping))

    if inverse:
        # Target
        targetPosition = [220.743, 463.375, 325.329, -0.5, -0.5, -0.5, 0.5]
        target = node.addChild('EffectorTarget')
        target.addObject('EulerImplicitSolver', firstOrder=True)
        target.addObject('CGLinearSolver', iterations=100, threshold=1e-2, tolerance=1e-5)
        target.addObject('MechanicalObject', name='dofs', template='Rigid3',
                         position=targetPosition,
                         showObject=True, drawMode=0, showObjectScale=10)
        target.addObject('UncoupledConstraintCorrection')

        if withEffector:
            # Effector
            index = 0 if withSoftPart else 6
            effector = softExtremity if withSoftPart else rigid
            effector.addObject('PositionEffector', name='effector', template='Rigid3', indices=index,
                               effectorGoal=target.dofs.findData('position').getLinkPath(),
                               useDirections=[1, 1, 1, 1, 1, 1])

        # Actuators
        for i in [0, 1, 2, 4, 5]:
            articulations.addObject('JointActuator', name='joint' + str(i), index=i, maxAngleVariation=0.001)

    else:
        articulations.addObject('RestShapeSpringsForceField', stiffness=1e10, points=list(range(6)))

    return robot


# Test/example scene
def createScene(rootNode):
    INVERSE = True  # Inverse kinematics problem solving (effector control). If True, INTERFACE and EXTERNALCONTROL will be ignored.
    INTERFACE = not INVERSE  # User interface to control the robot
    WITHSOFTPART = True  # Add soft part between the robot and the probe
    COMMUNICATION = False  # Send/receive joints' value
    EXTERNALCONTROL = False  # Use values received from an external software to control the robot (requires COMMUNICATION=True)

    from Header import addHeader
    addHeader(rootNode)
    rootNode.addObject('RequiredPlugin', name='ExternalPlugins',
                       pluginName=['SoftRobots', 'SoftRobots.Inverse' if INVERSE else ''])
    rootNode.addObject('FreeMotionAnimationLoop')
    if INVERSE:
        rootNode.addObject('QPInverseProblemSolver', maxIterations=1000, tolerance=1e-8, epsilon=0.001, printLog=False)
    else:
        rootNode.addObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-8, printLog=False)

    simulation = rootNode.addChild('Simulation')

    # Initial configuration
    # initAngles=[0,0,0,0,0,0]
    # initConfiguration = None

    # Change initial configuration
    initAngles = params.initAngles
    initConfiguration = [initAngles, params.initEffectorPosition + params.initEffectorOrientation]

    robot = createRobot(simulation, inverse=INVERSE, initConfiguration=initConfiguration, withSoftPart=WITHSOFTPART)

    if INTERFACE and not INVERSE:
        RobotGUI(robot=robot, initAngles=initAngles)

    if COMMUNICATION:
        # Option1: Communication through usb
        # rootNode.addObject('SerialPortBridgeGeneric', name='serial', port='/dev/ttyACM0',
        #                         baudRate=115200, size=5, listening=True, header=255,
        #                         packetOut=robot.findData('packetOut').getLinkPath()) # filled in RobotGUI.py

        # Option2: Communication using sockets
        # Requirements: SoftRobots/component/controller/README.md CommunicationController part

        # Send joints' value
        rootNode.addObject('CommunicationController', name='toExtSoftware', listening=True, job='sender', port='5558',
                           nbDataField=1, pattern=0, template='Vec1',
                           data1=robot.findData('angles').getLinkPath())
        # Receive joints' value
        rootNode.addObject('CommunicationController', name='fromExtSoftware', listening=True, job='receiver',
                           port='5559', nbDataField=1, pattern=0, template='Vec1',
                           data1=[0., 0., 0., 0., 0., 0.])
        # The vector we receive from the extertnal software should be a string,
        # containing the size of the vector and then the joints' value (e.g. "6 0. 0. 0. 0. 0. 0.")

    if EXTERNALCONTROL and COMMUNICATION and not INVERSE:
        robot.findData('angles').value = rootNode.fromExtSoftware.findData('data1').getLinkPath()

    return
