# encoding: utf-8
from __future__ import print_function
import os
import json
from .core import Scene
from stlib.physics.rigid import Floor


class ObstacleScene(Scene):
    """This class handles the scene creation of the robot model."""

    def init_scene(self):
        with open(os.path.join(self.asset_dir, "sim_params.json"), 'r') as f:
            self.sim_params = json.load(f)

        # required sim params
        self.dt = self.sim_params['dt']
        self.gravity = self.sim_params['gravity']
        self.with_gui = self.sim_params['with_gui']
        self.debug = self.sim_params['debug']
        self.friction_coef = self.sim_params['friction_coef']
        required_plugins = [
            'SoftRobots',
            'ModelOrderReduction',
        ]

        for i in required_plugins:
            self.root.createObject("RequiredPlugin", name='req_p' + i,
                                   pluginName=i)

        self.root.findData('gravity').value = self.gravity
        self.root.findData('dt').value = self.dt

        # create all the material and cavities
        self.root.createObject('FreeMotionAnimationLoop')
        self.root.createObject('GenericConstraintSolver', printLog='0',
                               tolerance="1e-8", maxIterations="250")

        self.root.createObject('DefaultPipeline', name='collisionPipeline',
                               verbose="0")
        self.root.createObject('BruteForceBroadPhase', name="BP")
        self.root.createObject('BVHNarrowPhase', name="NP")

        self.root.createObject('CollisionResponse', response="FrictionContact", responseParams="mu="+str(self.friction_coef))
        self.root.createObject('LocalMinDistance', name="Proximity",
                               alarmDistance="10.5", contactDistance="0.5",
                               angleCone="0.1")

        self.root.createObject('BackgroundSetting',
                               color='0 0.168627 0.211765')
        self.root.createObject('OglSceneFrame', style="Arrows",
                               alignment="TopRight")
        self.robot.load(self.root)


        #Insert obstacle file here
        planeNode = self.root.createChild('Plane')
        planeNode.createObject('MeshObjLoader', name='loader', filename='./scenes/Terrain/floorFlat.obj', triangulate=True, rotation=[90, 0, 0], scale=100, translation=[0, 0, -10])
        planeNode.createObject('MeshTopology', src='@loader')
        planeNode.createObject('MechanicalObject', src='@loader')
        planeNode.createObject('TriangleCollisionModel', simulated=False, moving=False)
        planeNode.createObject('LineCollisionModel', simulated=False, moving=False)
        planeNode.createObject('PointCollisionModel', simulated=False, moving=False)
        planeNode.createObject('OglModel',name='Visual', src='@loader', color=[1, 0, 0, 1])



    def act(self, action):
        self.robot.act(action)

    def observe(self):
        return self.robot.observe()
