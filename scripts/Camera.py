import Sofa
import Sofa.Core


def createCamera(rootNode, position, rigidIndex):
    Camera = rootNode.addChild('Camera')
    Camera.addObject('MechanicalObject', name="dofs", template='Rigid3d', position=position)
    Camera.addObject('RigidMapping', name="mapping", index=rigidIndex)

    CameraVisu = Camera.addChild('CameraVisu')
    CameraVisu.addObject('MeshOBJLoader', filename='mesh/cylinder.obj', name='loader', scale=2, rotation=[0, 0, 90])
    CameraVisu.addObject('OglModel', src='@loader', color=[0, 0, 0])
    CameraVisu.addObject('RigidMapping', name="mapping", index=0)

    return Camera


# Test scene
def createScene(rootNode):
    from Header import addHeader
    addHeader(rootNode)

    rigid = rootNode.addChild('Base')
    rigid.addObject('MechanicalObject', template='Rigid3', name='dofs',
                    position=[0, 0, 0, 0, 0, 0, 1], showObject=True,
                    showObjectScale=3)
    createCamera(rigid, [50, 0, 0, 0, 0, 0, 1], 0)
