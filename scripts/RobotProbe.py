from Robot import createRobot
from RobotGUI import RobotGUI
from NeedleProbe import createNeedleProbe, addProbe
from importlib.machinery import SourceFileLoader
import os

params = SourceFileLoader('params', os.getcwd() + '/SceneParams.py').load_module()


# Test/example scene
def createScene(rootNode):
    INVERSE = True  # Inverse kinematics problem-solving (effector control).
    # If True, INTERFACE and EXTERNALCONTROL will be ignored.
    INTERFACE = not INVERSE  # User interface to control the robot

    from Header import addHeader
    addHeader(rootNode)
    rootNode.addObject('RequiredPlugin', name='ExternalPlugins',
                       pluginName=['SoftRobots', 'BeamAdapter',
                                   'SoftRobots.Inverse' if INVERSE else ''])
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver' if INVERSE else 'GenericConstraintSolver', name='constraintsolver',
                       maxIterations=250, tolerance=1e-5, epsilon=1e-2)

    simulation = rootNode.addChild('Simulation')

    # Initial configuration
    # initAngles=[0,0,0,0,0,0]
    # initConfiguration = None
    # softExtremityPosition = [0,0,0,0,0,-0.707,0.707]

    # Change initial configuration
    initConfiguration = [params.initAngles, params.initEffectorPosition + params.initEffectorOrientation]
    softExtremityPosition = [0, 0,
                             0] + params.initSoftExtremityOrientation
    # BeamInterpolation acts weird when we translate the beam at init. TODO: find out why

    robot = createRobot(simulation, inverse=INVERSE, initConfiguration=initConfiguration, withEffector=False)
    addProbe(robot.SoftExtremity, 0)
    if INTERFACE:
        RobotGUI(robot, params.initAngles)
    if INVERSE:
        robot.SoftExtremity.ProbeColli.addObject('PositionEffector', name='effector', template='Rigid3', indices=0,
                                                 effectorGoal=simulation.EffectorTarget.dofs.findData(
                                                     'position').getLinkPath(),
                                                 useDirections=[1, 1, 1, 1, 1, 1])

    needleProbe = createNeedleProbe(simulation, 'NeedleProbe',
                                    withProbe=False,
                                    withNeedle=True,
                                    position=softExtremityPosition)
    needleProbe.EulerImplicitSolver.rayleighMass = 0
    needleProbe.EulerImplicitSolver.rayleighStiffness = 0

    for i in range(3):
        needleProbe.addObject('RestShapeSpringsForceField', template='Rigid3', name='attach' + str(i), points=i,
                              external_points=0, external_rest_shape=robot.SoftExtremity.dofs.getLinkPath(),
                              stiffness=1e12, angularStiffness=1e12)
    needleProbe.init()  # BeamInterpolation acts weird when we translate the beam at init. TODO: find out why
    needleProbe.dofs.translation = params.initSoftExtremityPosition
