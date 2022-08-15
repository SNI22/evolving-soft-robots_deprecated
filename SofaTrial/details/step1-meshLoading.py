#import Sofa


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaPython3 SofaLoader')

    disk = rootNode.addChild('disk')
    disk.addObject('MeshVTKLoader', name='loader', filename='data/mesh/mesh_generation/disk.vtk')
    disk.addObject('MeshTopology', src='@loader', name='container')
    disk.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1)
    pneuNet0000 = rootNode.addChild('pneuNet0000')
    pneuNet0000.addObject('MeshGmshLoader', name='loader', filename='data/mesh/mesh_generation/pneunet_0000/body.msh')
    pneuNet0000.addObject('MeshTopology', src='@loader', name='container')
    pneuNet0000.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1)

    pneuNet0000(rootNode, translation = [0,0,-20])