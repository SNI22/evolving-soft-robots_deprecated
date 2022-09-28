#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core
from Sofa.constants import *
import math


class graspingAttempt(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.constraints = []
        self.dofs = []
        for i in range(1,5):
            self.dofs.append(self.node.getChild('finger' + str(i)).tetras)
            self.constraints.append(self.node.getChild('finger'+str(i)).cavity.SurfacePressureConstraint)

        self.centerPosY = 70
        self.centerPosZ = 0
        self.rotAngle = 0

        return

      


    def onBeginAnimationStep (self, dt = 0.33):
        if self.centerPosY > 30:
            for i in range(4):
                results = moveRestPos(self.dofs[i].rest_position.value, -3.0, 0.0, 0.0)
                self.dofs[i].rest_position.value = results
        