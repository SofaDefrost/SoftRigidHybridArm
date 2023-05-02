from importlib.machinery import SourceFileLoader
import os

filename = os.getcwd() + '/scripts/SceneParams.py'
filename = filename.replace("/scripts/scripts", "/scripts")
params = SourceFileLoader('params', filename).load_module()


def addBones(node):
    bones = node.addChild('Bones')
    bones.addObject('SparseGridRamificationTopology', n=[25, 15, 10], fileTopology=params.bonesPath,
                    onlyInsideCells=True)
    bones.addObject('MechanicalObject', name='dofs', rotation=params.rotation, translation=params.translation)
    bones.addObject('UniformMass', totalMass=0.5)
    bones.addObject('HexahedronFEMForceField', youngModulus=1e6, poissonRatio=0, method="large",
                    updateStiffnessMatrix=False, printLog=False)
    bones.addObject('BarycentricMapping')

    visu = bones.addChild('Visu')
    visu.addObject('MeshOBJLoader', name='mesh', filename=params.bonesPath, rotation=params.rotation,
                   translation=params.translation)
    visu.addObject('OglModel', src='@mesh', color=[0.9, 0.75, 0.5, 1])
    visu.addObject('BarycentricMapping')

    return bones


def addTargetToBody(node, position):
    target = node.addChild('Target')
    target.addObject('MechanicalObject', name='dofs', position=position, showObject=True, showObjectScale=5, drawMode=2)
    target.addObject('BarycentricMapping')


def addLesionToBody(node, filename, name='Lesion', rotation=params.rotation, translation=params.translation):
    lesion = node.addChild(name)
    lesion.addObject('MeshSTLLoader', name='loader', filename=filename, rotation=rotation, translation=translation)
    lesion.addObject('OglModel', src='@loader', color=[0.2, 0.2, 0.2, 1])
    lesion.addObject('BarycentricMapping')


def addCollisionToBody(node, collisionFilename, name='Collision', rotation=params.rotation,
                       translation=params.translation):
    collision = node.addChild(name)
    collision.addObject('MeshOBJLoader', name='loader', filename=collisionFilename, rotation=rotation,
                        translation=translation)
    collision.addObject('TriangleSetTopologyContainer', name="Container", src="@loader")
    collision.addObject('MechanicalObject')
    collision.addObject('TriangleCollisionModel', group=1)
    collision.addObject('BarycentricMapping')


def fixBody(tissue, params):
    gridMin = [params.gridMin[i] + params.translation[i] for i in range(3)]
    gridMax = [params.gridMax[i] + params.translation[i] for i in range(3)]
    boxes = []
    boxes.append([gridMin[0], gridMax[1] - 20, gridMin[2], gridMax[0], gridMax[1], gridMax[2]])
    boxes.append([gridMin[0], gridMin[1], gridMin[2], gridMax[0], gridMin[1] + 20, gridMax[2]])
    boxes.append([gridMax[0] - 20, gridMin[1], gridMin[2], gridMax[0], gridMax[1], gridMax[2]])
    boxes.append([gridMin[0], gridMin[1], gridMin[2], gridMin[0] + 20, gridMax[1], gridMax[2]])
    tissue.addObject('BoxROI', box=boxes, drawBoxes=True)
    tissue.addObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness=1e5)


def createBody(node, params, rotation=params.rotation, translation=params.translation, withSolver=True,
               withBodyFixed=True, withBones=True):
    modelling = node.getRoot().getChild('Modelling')
    if modelling == None:
        modelling = node.getRoot().addChild('Modelling')

    bodyGrid = modelling.addChild('BodyGrid')
    gridMin = [params.gridMin[i] + params.translation[i] for i in range(3)]
    gridMax = [params.gridMax[i] + params.translation[i] for i in range(3)]
    bodyGrid.addObject('RegularGridTopology', n=[7, 7, 4], min=gridMin, max=gridMax, name='topology')
    bodyGrid.addObject('TetrahedronSetTopologyContainer', name='container', src='@topology')
    bodyGrid.addObject('TetrahedronSetTopologyModifier')
    bodyGrid.addObject('Hexa2TetraTopologicalMapping', input='@topology', output='@container', swapping='false')

    body = node.addChild('Body')
    if withSolver:
        body.addObject('EulerImplicitSolver')
        body.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
        body.addObject('GenericConstraintCorrection')

    tissue = body.addChild('Tissue')
    tissue.addObject('TetrahedronSetTopologyContainer', name="Container", src=bodyGrid.container.getLinkPath())
    tissue.addObject('MechanicalObject', name='dofs', template='Vec3')
    tissue.addObject('UniformMass', totalMass=3)
    tissue.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large',
                     poissonRatio=params.poissonRatioTissueGrid, youngModulus=params.youngModulusTissueGrid,
                     drawHeterogeneousTetra=True)
    if withBodyFixed:
        fixBody(tissue, params)

    prostate = tissue.addChild('Prostate')
    prostate.addObject('SparseGridRamificationTopology', n=[8, 8, 5], fileTopology=params.prostatePath, name='topology')
    prostate.addObject('MechanicalObject', name='dofs', template='Vec3d', rotation=params.rotation,
                       translation=params.translation)
    prostate.addObject('UniformMass', totalMass=0.5)
    prostate.addObject('HexahedronFEMForceField', youngModulus=params.youngModulusProstate,
                       poissonRatio=params.poissonRatioProstate, method="large", updateStiffnessMatrix=False,
                       printLog=False)
    prostate.addObject('BarycentricMapping')

    visuRectum = tissue.addChild('Rectum')
    visuRectum.addObject('MeshOBJLoader', name='mesh', filename=params.rectumPath, triangulate=True,
                         rotation=params.rotation, translation=params.translation)
    visuRectum.addObject('OglModel', src='@mesh', color=[0.5, 0., 0., 1])
    visuRectum.addObject('BarycentricMapping')

    visuprostate = prostate.addChild('VisuProstate')
    visuprostate.addObject('MeshOBJLoader', filename=params.prostatePath, triangulate=True, name='mesh',
                           rotation=params.rotation, translation=params.translation)
    visuprostate.addObject('OglModel', src='@mesh', color=[1.0, 0.5, 0.5, 1])
    visuprostate.addObject('BarycentricMapping')

    visuprostatecapsule = prostate.addChild('VisuProstateCapsule')
    visuprostatecapsule.addObject('MeshOBJLoader', filename=params.prostateCapsulePath, triangulate=True, name='mesh',
                                  rotation=params.rotation, translation=params.translation)
    visuprostatecapsule.addObject('OglModel', src='@mesh', color=[1.0, 0.5, 0.5, 1])
    visuprostatecapsule.addObject('BarycentricMapping')

    visuBladder = tissue.addChild('Bladder')
    visuBladder.addObject('MeshOBJLoader', name='mesh3', filename=params.bladderPath, triangulate=True,
                          rotation=params.rotation, translation=params.translation)
    visuBladder.addObject('OglModel', src='@mesh3', color=[1.0, 0.3, 0.3, 1])
    visuBladder.addObject('BarycentricMapping')

    if withBones:
        bones = addBones(tissue)

    return tissue


# Test scene
def createScene(rootNode):
    from Header import addHeader
    addHeader(rootNode)
    rootNode.VisualStyle.displayFlags = 'showBehavior showWireframe'

    tissue = createBody(rootNode, params, withSolver=True, withBodyFixed=True, withTrajectory=False)
    addCollisionToBody(tissue.Prostate, params.prostateColliPath, 'ProstateCollision')
    for i in range(4):
        addLesionToBody(tissue, params.lesionsPath[i], 'Lesion' + str(i), params.rotation, params.translation)
