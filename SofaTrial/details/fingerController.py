#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key


class FingerController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.node = args[0]
        self.fingerNode = self.node.getChild('finger')
        self.pressureConstraint = self.fingerNode.cavity.getObject('SurfacePressureConstraint')

    def onKeypressedEvent(self, e):
        pressureValue = self.pressureConstraint.value.value[0]

        if e["key"] == Key.plus:
            pressureValue += 1
            if pressureValue > 150:
                pressureValue = 150

        if e["key"] == Key.minus:
            pressureValue -= 0.01
            if pressureValue < 0:
                pressureValue = 0

        self.pressureConstraint.value = [pressureValue]
