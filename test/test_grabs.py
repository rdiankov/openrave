# -*- coding: utf-8 -*-

from common_test_openrave import *

class TestGrabs(EnvironmentSetup):

    def setup(self):
        EnvironmentSetup.setup(self)

    def test_basic_grab(self):
        with self.env:
            # Create two kinbodies
            grabber = RaveCreateKinBody(self.env, '')
            grabber.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabber.SetName('grabber')
            self.env.Add(grabber)

            grabbee = RaveCreateKinBody(self.env, '')
            grabbee.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabbee.SetName('grabbee')
            self.env.Add(grabbee)

            # Locate them
            grabberTransform = eye(4)
            grabberTransform[0:3,3] = [1, 2, 3]
            grabber.SetTransform(grabberTransform)

            grabbeeTransform = eye(4)
            grabbeeTransform[0:3,3] = [4, 5, 6]
            grabbee.SetTransform(grabbeeTransform)

            # Grab one with the other
            grabber.Grab(grabbee, grabber.GetLinks()[0])

            # Assert it is grabbed correctly
            grabLink = grabber.IsGrabbing(grabbee)
            assert(grabLink == grabber.GetLinks()[0])

            # Relative transform should be correct
            grabInfo = grabber.GetGrabbedInfo()[0]
            expectedRelative = dot(grabbeeTransform, linalg.inv(grabberTransform)) # Grabbee initial transform minus grabber initial transform
            assert(transdist(grabInfo._trelative, expectedRelative) < g_epsilon)

            # Ungrab it
            grabber.Release(grabbee)

            # Shouldn't be grabbed anymore
            assert(grabber.IsGrabbing(grabbee) is None)

    def test_grab_self(self):
        with self.env:
            # Create a kinbody
            grabber = RaveCreateKinBody(self.env, '')
            grabber.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabber.SetName('grabber')
            self.env.Add(grabber)

            # Attempting to grab it with itself should throw an exception
            with assert_raises(OpenRAVEException):
                grabber.Grab(grabber, grabber.GetLinks()[0])

    def test_grab_renamed_body(self):
        with self.env:
            # Create two kinbodies
            grabber = RaveCreateKinBody(self.env, '')
            grabber.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabber.SetName('grabber')
            self.env.Add(grabber)

            grabbee = RaveCreateKinBody(self.env, '')
            grabbee.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabbee.SetName('grabbee')
            self.env.Add(grabbee)

            # Grab one with the other
            grabber.Grab(grabbee, grabber.GetLinks()[0])

            # Assert it is grabbed correctly
            grabLink = grabber.IsGrabbing(grabbee)
            assert(grabLink == grabber.GetLinks()[0])

            # Rename the grabbed body
            grabbee.SetName('grabbee2')

            # Assert it is still grabbed
            grabLink = grabber.IsGrabbing(grabbee)
            assert(grabLink == grabber.GetLinks()[0])

    def test_grab_reset(self):
        with self.env:
            # Create two kinbodies
            grabber = RaveCreateKinBody(self.env, '')
            grabber.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabber.SetName('grabber')
            self.env.Add(grabber)

            grabbee = RaveCreateKinBody(self.env, '')
            grabbee.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabbee.SetName('grabbee')
            self.env.Add(grabbee)

            # Grab one with the other
            grabber.Grab(grabbee, grabber.GetLinks()[0])

            # Assert it is grabbed correctly
            grabLink = grabber.IsGrabbing(grabbee)
            assert(grabLink == grabber.GetLinks()[0])

            # Current grab transform should be identity
            grabInfo = grabber.GetGrabbedInfo()[0]
            assert(transdist(grabInfo._trelative, eye(4)) < g_epsilon)

            # Reset the grab using a grabinfo with different transform
            grabInfo = grabber.GetGrabbedInfo()[0]
            grabInfo._trelative[0:3,3] = [1, 2, 3]
            grabber.ResetGrabbed([grabInfo])

            # Assert that it is still grabbed
            grabLink = grabber.IsGrabbing(grabbee)
            assert(grabLink == grabber.GetLinks()[0])

            # Transform of the body in the env should be updated
            assert(transdist(grabbee.GetTransform(), grabInfo._trelative) < g_epsilon)

    def test_grabbed_object_transform(self):
        with self.env:
            # Create two kinbodies
            grabber = RaveCreateKinBody(self.env, '')
            grabber.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabber.SetName('grabber')
            self.env.Add(grabber)

            grabbee = RaveCreateKinBody(self.env, '')
            grabbee.InitFromBoxes(array([[0,0,0,1,1,1]]), True)
            grabbee.SetName('grabbee')
            self.env.Add(grabbee)

            # Grab one with the other
            grabber.Grab(grabbee, grabber.GetLinks()[0])

            # Assert it is grabbed correctly
            grabLink = grabber.IsGrabbing(grabbee)
            assert(grabLink == grabber.GetLinks()[0])

            # Both bodies should be at origin, relative transform should be identity
            grabInfo = grabber.GetGrabbedInfo()[0]
            assert(transdist(grabber.GetTransform(), eye(4)) < g_epsilon)
            assert(transdist(grabbee.GetTransform(), eye(4)) < g_epsilon)
            assert(transdist(grabInfo._trelative, eye(4)) < g_epsilon)

            # Set transform of first body
            baseTransform = eye(4)
            baseTransform[0:3,3] = [1, 2, 3]
            grabber.SetTransform(baseTransform)

            # Transform of grabbed body should have also updated
            grabInfo = grabber.GetGrabbedInfo()[0]
            assert(transdist(grabber.GetTransform(), baseTransform) < g_epsilon)
            assert(transdist(grabbee.GetTransform(), baseTransform) < g_epsilon)
            assert(transdist(grabInfo._trelative, eye(4)) < g_epsilon)
