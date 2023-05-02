from scripts.Body import createBody, addCollisionToBody, addTargetToBody
from scripts.Header import addHeader
from scripts.NeedleProbe import addProbe
from scripts.Robot import createRobot
from splib3.numerics import vsub

from importlib.machinery import SourceFileLoader
import os

params = SourceFileLoader('params', os.getcwd() + '/scripts/SceneParams.py').load_module()


# Test to optimize the alignment of the probe with a target point in the prostate
# We should be able to couple the effectors... meaning: targetpoint = probeeffectorshifted
# In the scenario below it is not coupled
def createScene(rootNode):

    addHeader(rootNode)
    rootNode.addObject('RequiredPlugin', name='ExternalPlugins',
                       pluginName=['SoftRobots', 'SoftRobots.Inverse'])

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog=0, tolerance=1e-8, maxIterations=1000, epsilon=0.0001,
                       minContactForces=0)
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('DefaultContactManager', response='FrictionContactConstraint', responseParams='mu=0.0')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('NewProximityIntersection', alarmDistance=3, contactDistance=0.1)
    rootNode.addObject('OglViewport', screenSize=[450, 450],
                       cameraPosition=[-55, -6, 31],
                       cameraOrientation=[0.0218681, -0.724597, 0.024092, 0.688405], useFBO=False)

    simulation = rootNode.addChild('Modelling')
    simulation = rootNode.addChild('Simulation')
    simulation.addObject('EulerImplicitSolver')
    simulation.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
    simulation.addObject('GenericConstraintCorrection')

    # Tissues prostate
    prostate = createBody(simulation, params, withSolver=False, withBodyFixed=True)
    addCollisionToBody(prostate, params.prostateColliPath, 'ProstateCollision')
    addCollisionToBody(prostate, params.rectumColliPath, 'RectumCollision')
    addTargetToBody(prostate, [10, -8, 50])
    prostate.addObject('VisualStyle', displayFlags='showWireframe')

    # Robot probe
    robotBasePosition = [-182.5, -573, -278]
    initAngles = [-0.09, -0.8158, 0.1267, 0, -0.937, 0]
    initEffectorPosition = [6.13285, -86.2749, -271.999, -0.766295, 0.0343984, -0.0306686, 0.640834]
    initEffectorPosition[0:3] = vsub(initEffectorPosition[0:3], [-182.5, -570, -285])
    robotInitConfiguration = [initAngles, initEffectorPosition]
    robot = createRobot(simulation, translation=robotBasePosition, initConfiguration=robotInitConfiguration,
                        inverse=True, withEffector=False, withSolver=False)
    robot.Articulations.joint0.maxAngle = 0
    robot.Articulations.joint0.minAngle = 0
    robot.Articulations.joint5.maxAngle = 0
    robot.Articulations.joint5.minAngle = 0
    simulation.removeChild(simulation.EffectorTarget)
    softPartExtremity = robot.SoftExtremity
    addProbe(softPartExtremity, 0)

    # Target
    target = softPartExtremity.addChild('Target')
    target.addObject('MechanicalObject', name='dofs', template='Vec3', position=[230, 0, 0], showObject=False,
                     showObjectScale=2, drawMode=1)
    target.addObject('RigidMapping', name='mapping', index=0)

    softPartExtremity.Target.addObject('PositionEffector', indices=0, name='effector',
                                       useDirections=[1, 1, 1],
                                       effectorGoal=prostate.Target.dofs.findData('position').getLinkPath())
