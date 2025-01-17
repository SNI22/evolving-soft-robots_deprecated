from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
import Sofa
import math
import os
from wholeGripperController import WholeGripperController
from animationLoop import graspingAttempt
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

youngModulusFingers = 500
youngModulusStiffLayerFingers = 1500
poissonRatioFingers = 0.3
fingersMass = 0.04

radius = 70
angle1 = 90*math.pi/180  # Angle between 1st and 2nd finger in radian
angle2 = 180*math.pi/180  # Angle between 1st and 3rd finger in radian
angle3 = 270*math.pi/180  # Angle between 1st and 4th finger in radian

translateFinger1 = "0 0 0"
translateFinger2 = "0 " + str(radius + radius*math.sin(angle1-math.pi/2)) + " " + str(radius*math.cos(angle1-math.pi/2))
translateFinger3 = "0 " + str(radius + radius*math.sin(angle2-math.pi/2)) + " " + str(radius*math.cos(angle2-math.pi/2))
translateFinger4 = "0 " + str(radius + radius*math.sin(angle3-math.pi/2)) + " " + str(radius*math.cos(angle3-math.pi/2))
translations = [translateFinger1, translateFinger2, translateFinger3, translateFinger4]
angles = [0, angle1, angle2, angle3]

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin',
                       pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver SofaMeshCollision SofaRigid SofaOpenglVisual')

    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    rootNode.gravity.value = [-9810, 0, 0];
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='FrictionContact', responseParams='mu=0.6')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=8, contactDistance=3, angleCone=0.0)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')

    planeNode = rootNode.addChild('Plane')
    planeNode.addObject('MeshObjLoader', name='loader', filename='details/data/mesh/floorFlat.obj', triangulate=True,
                        rotation=[0, 0, 270], scale=10, translation=[-122, 0, 0])
    planeNode.addObject('MeshTopology', src='@loader')
    planeNode.addObject('MechanicalObject', src='@loader')
    planeNode.addObject('TriangleCollisionModel', simulated=False, moving=False)
    planeNode.addObject('LineCollisionModel', simulated=False, moving=False)
    planeNode.addObject('PointCollisionModel', simulated=False, moving=False)
    planeNode.addObject('OglModel', name='Visual', src='@loader', color=[1, 0, 0, 1])

    cube = rootNode.addChild('cube')
    cube.addObject('EulerImplicitSolver', name='odesolver')
    cube.addObject('SparseLDLSolver', name='linearSolver')
    cube.addObject('MechanicalObject', template='Rigid3', position=[-70, 70, 0, 0, 0, 0, 1])
    cube.addObject('UniformMass', totalMass=0.001)
    cube.addObject('UncoupledConstraintCorrection')

    # collision
    cubeCollis = cube.addChild('cubeCollis')
    cubeCollis.addObject('MeshObjLoader', name='loader', filename='details/data/mesh/smCube27.obj', triangulate=True,
                         scale=7)
    cubeCollis.addObject('MeshTopology', src='@loader')
    cubeCollis.addObject('MechanicalObject')
    cubeCollis.addObject('TriangleCollisionModel')
    cubeCollis.addObject('LineCollisionModel')
    cubeCollis.addObject('PointCollisionModel')
    cubeCollis.addObject('RigidMapping')

    # visualization
    cubeVisu = cube.addChild('cubeVisu')
    cubeVisu.addObject('MeshObjLoader', name='loader', filename='details/data/mesh/smCube27.obj')
    cubeVisu.addObject('OglModel', name='Visual', src='@loader', color=[0.0, 0.1, 0.5], scale=8)
    cubeVisu.addObject('RigidMapping')

    for i in range(4):
        ##########################################
        # Finger Model	 						 #
        ##########################################
        finger = rootNode.addChild('finger' + str(i + 1))
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner')


        if i == 0:
            finger.addObject('MeshGmshLoader', name='loader', filename='meshes/body9.msh',
            rotation=[360 - angles[i] * 180 / math.pi, 0, 0], translation=translations[i])
            finger.addObject('MeshTopology', src='@loader', name='container')

            finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
            finger.addObject('UniformMass', totalMass=0.04)
            finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                            youngModulus=youngModulusFingers)

            finger.addObject('BoxROI', name='boxROI', box=[40, -15, -5, 60, 15, 20], doUpdate=False )
            finger.addObject('BoxROI', name='boxROISubTopo', box=[-60, -4, -20, 60, 0, 20], strict=False, drawBoxes =False, drawSize = 1) #
            finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12,
                             angularStiffness=1e12)
        elif i == 1:
            finger.addObject('MeshGmshLoader', name='loader', filename='meshes/body11.msh',
            rotation=[360 - angles[i] * 180 / math.pi, 0, 0], translation=translations[i])
            finger.addObject('MeshTopology', src='@loader', name='container')

            finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
            finger.addObject('UniformMass', totalMass=0.04)
            finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                            youngModulus=youngModulusFingers)

            finger.addObject('BoxROI', name='boxROI', box=[40, 55, 65, 60, 85, 90], doUpdate=False, drawBoxes = False )
            finger.addObject('BoxROI', name='boxROISubTopo', box=[-80, 50, 78, 60, 90, 74], strict=False, drawBoxes =False, drawSize = 1) #
            finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12,
                             angularStiffness=1e12)
            # finger.addObject('RestShapeSpringsForceField', points='@.boxROI.indices', stiffness=1e12,
            #                  angularStiffness=1e12)
        elif i ==2:
            finger.addObject('MeshGmshLoader', name='loader', filename='meshes/body9.msh',
            rotation=[360 - angles[i] * 180 / math.pi, 0, 0], translation=translations[i])
            finger.addObject('MeshTopology', src='@loader', name='container')

            finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
            finger.addObject('UniformMass', totalMass=0.04)
            finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                            youngModulus=youngModulusFingers)

            # finger.addObject('BoxROI', name='boxROI', box=[40, -15, -5, 60, 15, 20], doUpdate=False )
            # finger.addObject('BoxROI', name='boxROISubTopo', box=[-60, -4, -20, 60, 0, 20], strict=False, drawBoxes =False, drawSize = 1) #
            # # finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12,
            # #                  angularStiffness=1e12)
            finger.addObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness=1e12,
                             angularStiffness=1e12)
        elif i ==3:
            finger.addObject('MeshGmshLoader', name='loader', filename='meshes/body11.msh',
            rotation=[360 - angles[i] * 180 / math.pi, 0, 0], translation=translations[i])
            finger.addObject('MeshTopology', src='@loader', name='container')

            finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
            finger.addObject('UniformMass', totalMass=0.04)
            finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                            youngModulus=youngModulusFingers)

            finger.addObject('BoxROI', name='boxROI', box=[40, -15, -5, 60, 15, 20], doUpdate=False )
            finger.addObject('BoxROI', name='boxROISubTopo', box=[-60, -4, -20, 60, 0, 20], strict=False, drawBoxes =False, drawSize = 1) #
            finger.addObject('RestShapeSpringsForceField', points='@../finger2/boxROI.indices', stiffness=1e12,
                             angularStiffness=1e12)

        finger.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')

        ##########################################
        # Sub topology						   #      #tetrahedric tology error
        ##########################################
        modelSubTopo = finger.addChild('modelSubTopo')
        if i == 0:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position',
                                   tetrahedra='@boxROISubTopo.tetrahedraInROI', name='container')
        elif i == 1:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position',
                                   tetrahedra='@boxROISubTopo.tetrahedraInROI', name='container')
        elif i == 2:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position',
                                   tetrahedra='@../../finger1/boxROISubTopo.tetrahedraInROI', name='container')
        elif i == 3:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position',
                                   tetrahedra='@../../finger2/boxROISubTopo.tetrahedraInROI', name='container')          

        modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large',
                               poissonRatio=0.3, youngModulus=youngModulusStiffLayerFingers - youngModulusFingers)


        if i == 0 or i == 2:
            ##########################################
            # Constraint							 #
            ##########################################
            cavity = finger.addChild('cavity')
            cavity.addObject('MeshSTLLoader', name='loader', filename='meshes/cavity9.stl',
                            translation=translations[i], rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
            cavity.addObject('MeshTopology', src='@loader', name='topo')
            cavity.addObject('MechanicalObject', name='cavity')
            cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0001,
                            triangles='@topo.triangles', valueType='pressure')
            cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

            ##########################################
            # Collision							  #
            ##########################################

            collisionFinger = finger.addChild('collisionFinger')
            collisionFinger.addObject('MeshSTLLoader', name='loader', filename='meshes/collision9.stl',
                                    translation=translations[i], rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
            collisionFinger.addObject('MeshTopology', src='@loader', name='topo')
            collisionFinger.addObject('MechanicalObject', name='collisMech')
            collisionFinger.addObject('TriangleCollisionModel', selfCollision=False)
            collisionFinger.addObject('LineCollisionModel', selfCollision=False)
            collisionFinger.addObject('PointCollisionModel', selfCollision=False)
            collisionFinger.addObject('BarycentricMapping')

            ##########################################
            # Visualization						  #
            ##########################################
            modelVisu = finger.addChild('visu')
            modelVisu.addObject('MeshGmshLoader', name='loader', filename='meshes/body9.msh')
            modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6], translation=translations[i],
                                rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
            modelVisu.addObject('BarycentricMapping')
        if i == 1 or i == 3:
            ##########################################
            # Constraint							 #
            ##########################################
            cavity = finger.addChild('cavity')
            cavity.addObject('MeshSTLLoader', name='loader', filename='meshes/cavity11.stl',
                            translation=translations[i], rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
            cavity.addObject('MeshTopology', src='@loader', name='topo')
            cavity.addObject('MechanicalObject', name='cavity')
            cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0001,
                            triangles='@topo.triangles', valueType='pressure')
            cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

            ##########################################
            # Collision							  #
            ##########################################

            collisionFinger = finger.addChild('collisionFinger')
            collisionFinger.addObject('MeshSTLLoader', name='loader', filename='meshes/collision11.stl',
                                    translation=translations[i], rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
            collisionFinger.addObject('MeshTopology', src='@loader', name='topo')
            collisionFinger.addObject('MechanicalObject', name='collisMech')
            collisionFinger.addObject('TriangleCollisionModel', selfCollision=False)
            collisionFinger.addObject('LineCollisionModel', selfCollision=False)
            collisionFinger.addObject('PointCollisionModel', selfCollision=False)
            collisionFinger.addObject('BarycentricMapping')

            ##########################################
            # Visualization						  #
            ##########################################
            modelVisu = finger.addChild('visu')
            modelVisu.addObject('MeshGmshLoader', name='loader', filename='meshes/body11.msh')
            modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6], translation=translations[i],
                                rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
            modelVisu.addObject('BarycentricMapping')
            

    rootNode.addObject(WholeGripperController(node=rootNode))

    return rootNode