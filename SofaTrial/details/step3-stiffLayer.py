#import Sofa


def createScene(rootNode):
    rootNode.addObject('VisualStyle', displayFlags='showForceFields')
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine')
    rootNode.findData('gravity').value = [-9810, 0, 0];

    pneuNet = rootNode.addChild('   pneuNet')
    pneuNet.addObject('MeshVTKLoader', name='loader', filename='data/mesh/mesh_generation/pneunet_0000/body.vtk')
    pneuNet.addObject('MeshTopology', src='@loader', name='container')
    pneuNet.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1)
    pneuNet.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=500)
    pneuNet.addObject('UniformMass', totalMass=0.0008)
    pneuNet.addObject('BoxROI', name='boxROISubTopo', box=[-100, 22.5, -8, -19, 28, 8], drawBoxes=True, strict=False)

    modelSubTopo = pneuNet.addChild('modelSubTopo')
    modelSubTopo.addObject('MeshTopology', position='@loader.position', tetrahedra='@boxROISubTopo.tetrahedraInROI',
                           name='container')
    modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                           youngModulus=1500)
