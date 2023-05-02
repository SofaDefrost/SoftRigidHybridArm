from math import sin, cos, pi
from splib3.numerics.vec3 import Vec3, vadd
from splib3.numerics.quat import Quat

from importlib.machinery import SourceFileLoader
import os

filename = os.getcwd() + '/scripts/SceneParams.py'
filename = filename.replace("/scripts/scripts", "/scripts")
params = SourceFileLoader('params', filename).load_module()


# The soft part consists of a hollow cylinder with the following
# characteristics:
# diameter = 0.045 m
# thickness = 0.0030 m
# length = 0.10m
# material: Dragon-skin 30
# bends approximately 45 degrees under external loads (a tip force of 10N).

# Soft cylinder (silicone)
def addSoftCylinder(node, name, rigidNode1, rigidNode2, rigidPosition, translation,
                    radius=22.5,
                    length=100.,
                    thickness=15.):
    drawDebug = False

    rigidPosition[0:3] = vadd(rigidPosition[0:3], translation)

    v = Vec3([length, 0., 10.])
    v.rotateFromQuat(Quat(rigidPosition[3:7]))
    rigidPositions = [rigidPosition, vadd(v, rigidPosition[0:3]) + rigidPosition[3:7]]
    extremities = [rigidPositions[0][0:3], rigidPositions[1][0:3]]

    nbSubSections = 10  # FEM mesh resolution in longitudinal direction
    positions = [extremities[0]]
    direction = Vec3(1., 0., 0.)
    direction.rotateFromQuat(Quat(rigidPositions[0][3:7]))
    direction.normalize()
    direction.scale(length / nbSubSections)
    for i in range(1, nbSubSections + 1):
        positions += [[extremities[0][k] + direction[k] * i for k in range(3)]]

    modelling = node.getRoot().getChild('Modelling')
    if modelling is None:
        modelling = node.getRoot().addChild('Modelling')

    model = modelling.addChild(name + 'FullModel')
    edgetopo = model.addChild('EdgeModel')
    edgetopo.addObject('EdgeSetTopologyContainer', position=positions, edges=[[k, k + 1] for k in range(nbSubSections)])
    edgetopo.addObject('EdgeSetTopologyModifier')
    edgetopo.addObject('MechanicalObject', template='Rigid3',
                       position=[positions[k] + rigidPositions[0][3:7] for k in range(nbSubSections + 1)])

    quadtopo = edgetopo.addChild('QuadModel')
    quadtopo.addObject('QuadSetTopologyContainer')
    quadtopo.addObject('QuadSetTopologyModifier')
    quadtopo.addObject('MechanicalObject')
    quadtopo.addObject('Edge2QuadTopologicalMapping',
                       input=edgetopo.EdgeSetTopologyContainer.getLinkPath(),
                       output=quadtopo.QuadSetTopologyContainer.getLinkPath(),
                       flipNormals=True, nbPointsOnEachCircle=8, radius=radius)

    cylinder = quadtopo.addChild(name)
    cylinder.addObject('VisualStyle', displayFlags='showForceFields')
    cylinder.addObject('ExtrudeQuadsAndGenerateHexas', name='extruder',
                       surfaceVertices=quadtopo.MechanicalObject.findData('position').getLinkPath(),
                       surfaceQuads=quadtopo.QuadSetTopologyContainer.findData('quads').getLinkPath(),
                       thicknessIn=thickness, thicknessOut=0, numberOfSlices=1, flipNormals=True)
    cylinder.addObject('HexahedronSetTopologyContainer', name='container',
                       points=cylinder.extruder.findData('extrudedVertices').getLinkPath(),
                       hexahedra=cylinder.extruder.findData('extrudedHexas').getLinkPath())
    cylinder.addObject('HexahedronSetTopologyModifier')
    cylinder.addObject('MechanicalObject', name='dofs')
    cylinder.addObject('HexahedronFEMForceField', poissonRatio=params.poissonRatioRobot,
                       youngModulus=params.youngModulusRobot)
    p = rigidPosition[0:3]
    q = Quat(rigidPosition[3:7])
    x = Vec3(1., 0., 0.)
    x.rotateFromQuat(q)
    x.normalize()
    y = Vec3(0., 1., 0.)
    y.rotateFromQuat(q)
    y.normalize()
    z = Vec3(0., 0., 1.)
    z.rotateFromQuat(q)
    z.normalize()
    radiusBox = radius + 5
    p0 = [p[i] + y[i] * radiusBox + z[i] * radiusBox for i in range(3)]
    p1 = [p[i] + y[i] * radiusBox - z[i] * radiusBox for i in range(3)]
    p2 = [p[i] - y[i] * radiusBox - z[i] * radiusBox for i in range(3)]
    box1 = [p0, p1, p2, 5]
    p0 = [p0[i] + x[i] * length for i in range(3)]
    p1 = [p1[i] + x[i] * length for i in range(3)]
    p2 = [p2[i] + x[i] * length for i in range(3)]
    box2 = [p0, p1, p2, 5]
    cylinder.addObject('BoxROI', name='rigidROI1', orientedBox=box1, drawBoxes=drawDebug)
    cylinder.addObject('BoxROI', name='rigidROI2', orientedBox=box2, drawBoxes=drawDebug)
    cylinder.addObject('BoxROI', name='rigidROIAll', box=[box1, box2])
    model.init()
    cylinder.addObject('UniformMass', totalMass=0.2)

    deformable = node.addChild('DeformablePart')
    deformable.addObject('MechanicalObject', name='dofs', showObject=False, showObjectScale=2, drawMode=1,
                         showColor=[0., 0., 1., 1.])

    rigidified1 = rigidNode1.addChild('RigidifiedNodes1')
    rigidified1.addObject('MechanicalObject', name='dofs', showObject=drawDebug, showObjectScale=2, drawMode=1,
                          showColor=[1., 0., 0., 1.])

    rigidNode2.dofs.position = [rigidPositions[1]]
    rigidified2 = rigidNode2.addChild('RigidifiedNodes2')
    rigidified2.addObject('MechanicalObject', name='dofs', showObject=drawDebug, showObjectScale=2, drawMode=1,
                          showColor=[1., 0., 0., 1.])

    nbNodes = cylinder.dofs.size.value
    indexPairs = [[0, 0] for k in range(nbNodes)]
    indexDeformable = 0
    indexRigidified1 = 0
    indexRigidified2 = 0
    indicesOut = []
    indicesIn1 = []
    indicesIn2 = []
    for i in range(nbNodes):
        if [i] in cylinder.rigidROI1.indices.value:
            indexPairs[i][0] = 0
            indexPairs[i][1] = indexRigidified1
            indexRigidified1 += 1
            indicesIn1 += [i]
        elif [i] in cylinder.rigidROI2.indices.value:
            indexPairs[i][0] = 1
            indexPairs[i][1] = indexRigidified2
            indexRigidified2 += 1
            indicesIn2 += [i]
        else:
            indexPairs[i][0] = 2
            indexPairs[i][1] = indexDeformable
            indexDeformable += 1
            indicesOut += [i]

    rigidified1.dofs.position = [cylinder.dofs.position[i] for i in indicesIn1]
    rigidified1.addObject('RigidMapping', globalToLocalCoords=True)
    rigidified1.addChild(cylinder)
    rigidified2.dofs.position = [cylinder.dofs.position[i] for i in indicesIn2]
    rigidified2.addObject('RigidMapping', globalToLocalCoords=True)
    rigidified2.addChild(cylinder)
    deformable.dofs.position = [cylinder.dofs.position[i] for i in indicesOut]
    deformable.addChild(cylinder)
    cylinder.addObject('SubsetMultiMapping', template="Vec3,Vec3",
                       input=[rigidified1.dofs.getLinkPath(), rigidified2.dofs.getLinkPath(),
                              deformable.dofs.getLinkPath()],
                       output=cylinder.dofs.getLinkPath(),
                       indexPairs=indexPairs)

    return cylinder


# Create soft part node
def addSoftPart(node, rigidNode1, rigidNode2, rigidPosition, translation=[0., 0., 0.], name='SoftPart'):
    softPart = addSoftCylinder(node, name, rigidNode1, rigidNode2, rigidPosition, translation)
    return softPart


# Test example
def createScene(rootNode):
    from Header import addHeader
    addHeader(rootNode)
    rootNode.gravity = [0, 0, -9810]
    rootNode.VisualStyle.displayFlags = 'showForceFields'

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=500)

    simulation = rootNode.addChild('Simulation')
    simulation.addObject('EulerImplicitSolver')
    simulation.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
    simulation.addObject('GenericConstraintCorrection')

    rigidPosition = [10., 11., -20., 0., 0, sin(-pi / 3), cos(-pi / 3)]

    rigid1 = simulation.addChild('Rigid1')
    rigid1.addObject('MechanicalObject', name='dofs', template='Rigid3', position=rigidPosition, showObject=True,
                     showObjectScale=20, translation=[10, 10, 10])
    rigid1.addObject('FixedConstraint', indices=0)

    rigid2 = simulation.addChild('Rigid2')
    rigid2.addObject('MechanicalObject', name='dofs', template='Rigid3', showObject=True, showObjectScale=20)

    addSoftPart(simulation, rigid1, rigid2, rigidPosition, translation=[10, 10, 10], name='SoftCylinder')

    return
