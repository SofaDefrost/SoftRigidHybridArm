from splib3.numerics import Quat, Vec3
import os

path = os.getcwd() + '/../data/'
if not os.path.isdir(path):
    path = os.getcwd() + '/data/'

# parameters
length = 200.  # 20 cm
radius = 1  # 18G 1.27mm 17G 1.47mm (diameter)
YoungModulus = 119e9  # Titanium 119GPa
VolumeMass = 4.506e-3  # 4.506 g/cm^ 3 Titanium (Wikipedia)  soit 4.506e3 kg/m^3
# numerical parameters
numFrames = 4


def addProbe(node, index, translation=[0, -1, 8.5]):
    visuProbe = node.addChild('ProbeVisu')
    visuProbe.addObject('MeshSTLLoader', filename=path + 'usProbe.stl', name='loader', translation=translation,
                        rotation=[180, 0, 0])
    visuProbe.addObject('OglModel', src='@loader', color=[1, 1, 1])
    visuProbe.addObject('RigidMapping', name='mapping', index=index)

    colliProbe = node.addChild('ProbeColli')
    colliProbe.addObject('MechanicalObject', template='Rigid3',
                         position=[200 + translation[0], 1 + translation[1], -9.1 + translation[2]] + [0, 0, 0, 1],
                         showObject=False, showObjectScale=10)
    colliProbe.addObject('SphereCollisionModel', radius=6)
    colliProbe.addObject('RigidMapping', name='mapping', index=index)


def attachToExternalRestShape(node, needleProbe, position):
    needleRestShape = node.addChild('NeedleRestShape')
    needleRestShape.addObject('MechanicalObject', name='dofs', template='Rigid3',
                              position=[position for i in range(3)],
                              showObject=False, showObjectScale=10, drawMode=0)
    for k in range(3):
        needleProbe.addObject('RestShapeSpringsForceField', template='Rigid3', name='attach' + str(k), points=k,
                              external_points=k, external_rest_shape=needleRestShape.dofs.getLinkPath(),
                              stiffness=1e12, angularStiffness=1e12,
                              drawSpring=False)


def createNeedleProbe(rootNode, name='Needle', position=[0, 0, 0, 0, 0, 0, 1],
                      withNeedle=True, withProbe=True, withSolver=True):
    edgeSampling = []
    quat = Quat(position[3:7])
    direction = Vec3([1., 0., 0.])
    direction.rotateFromQuat(quat)
    edgeSampling.append(position[0:3])  # extra point for gunshot motion
    edgeSampling.append(position[0:3])  # extra point for intersection dof
    for i in range(numFrames):
        edgeSampling.append([position[k] + direction[k] * float(i * length) / float(numFrames - 1.) for k in range(3)])

    object = rootNode.addChild(name)
    if withSolver:
        object.addObject('EulerImplicitSolver', rayleighMass=1, rayleighStiffness=1)
        object.addObject('BTDLinearSolver', name='preconditioner')
        object.addObject('ShewchukPCGLinearSolver', iterations=2000, tolerance=1.0e-18,
                         preconditioner='@preconditioner')
        object.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
    objectEdgeContainer = object.addObject('EdgeSetTopologyContainer', name='beamtopology',
                                           edges=[[k, k + 1] for k in range(numFrames + 1)], position=edgeSampling)
    object.addObject('EdgeSetTopologyModifier', name='EdgeSetTopologyModifier')
    object.addObject('MechanicalObject', template='Rigid3', name='dofs', translation=[0, 0, 0],
                     position=[edgeSampling[k] + position[3:7] for k in range(numFrames + 2)],
                     showObject=False, showObjectScale=10)
    object.addObject('BeamInterpolation', name='beamInterpolation', defaultYoungModulus=YoungModulus,
                     dofsAndBeamsAligned=False,
                     DOF0TransformNode0=[[0, 0, 0, 0, 0, 0, 1] for i in range(5)],
                     DOF1TransformNode1=[[0, 0, 0, 0, 0, 0, 1] for i in range(5)],
                     straight=True, radius=radius, innerRadius=0.5, crossSectionShape='circular',
                     defaultPoissonRatio=0.32)
    object.addObject('AdaptiveBeamForceFieldAndMass', name='forcefield', computeMass=True, massDensity=VolumeMass)

    if withNeedle:
        localPositions = [Vec3(edgeSampling[i]) for i in range(numFrames + 2)]
        quat = Quat(position[3:7])
        quat = quat.getInverse()
        for i in range(numFrames + 2):
            localPositions[i].translate([-position[k] for k in range(3)])
            localPositions[i].rotateFromQuat(quat)
        localPositions = [localPositions[i].toList() for i in range(numFrames + 2)]

        needleColli = object.addChild('NeedleColli')
        needleColli.addObject('EdgeSetTopologyContainer', name='EdgeSetTopologyContainer',
                              position=localPositions,
                              edges=[[k, k + 1] for k in range(numFrames + 1)])
        needleColli.addObject('EdgeSetTopologyModifier', name='EdgeSetTopologyModifier')
        needleColli.addObject('MechanicalObject', name='dofs')
        needleColli.addObject('AdaptiveBeamMapping', interpolation='@../beamInterpolation', input='@../dofs',
                              output='@./dofs', useCurvAbs=True)

    if withProbe:
        addProbe(object, 0)

    return object


# Test/example scene
def createScene(rootNode):
    from Header import addHeader
    addHeader(rootNode)
    rootNode.addObject('RequiredPlugin', name='ExternalPlugins',
                       pluginName=['SoftRobots', 'BeamAdapter'])
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=250, tolerance=1e-5)

    simulation = rootNode.addChild('Simulation')

    orientation = [0, 0, 0, 1]
    position = [10, 0, 0]
    needleProbe = createNeedleProbe(simulation, 'NeedleProbe',
                                    withProbe=True,
                                    withNeedle=True,
                                    position=[0, 0, 0] + orientation)
    attachToExternalRestShape(simulation, needleProbe, position + orientation)
    # To translate...
    needleProbe.init()  # BeamInterpolation acts weird when we translate the beam at init. TODO: find out why
    needleProbe.dofs.translation = position
