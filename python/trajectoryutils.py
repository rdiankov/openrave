# -*- coding: utf-8 -*-
# Copyright (C) 2020 MUJIN Inc

import struct
import numpy

class BinaryTrajectory(object):
    """Represents a parsed binary trajectory
    """

    MAGIC_NUMBER = 0x62ff
    BINARY_TRAJECTORY_VERSION_NUMBER = 3

    class ConfigurationSpecificationGroup(object):
        """Represents a configuration spec group inside the binary trajectory
        """
        name = None # type: str # name of the group
        offset = None # type: int
        dof = None # type: int
        interpolation = None # type: str

    description = None # type: str
    groups = None # list of ConfigurationSpecificationGroup objects describing each group
    waypoints = None # type: numpy.array
    readableInterfaces = None # list of dict describing each readableInterface


def _ParseBinaryString(data, offset=0):
    """
    :return: (string, offset)
    """
    stringLength = struct.unpack_from('<H', data, offset=offset)[0]
    offset += struct.calcsize('<H')

    fmt = '<%ds' % stringLength
    stringValue = struct.unpack_from(fmt, data, offset=offset)[0]
    offset += struct.calcsize(fmt)
    return stringValue, offset

def ParseBinaryTrajectory(data, offset=0):
    """Parse binary trajectory file content

    :param data: content of the file, could be bytes or str
    :return: (a Trajectory object, offset)
    """
    magicNumber, versionNumber = struct.unpack_from('<HH', data, offset=offset)
    offset += struct.calcsize('<HH')

    if magicNumber != 0x62ff:
        raise ValueError('trajectory file did not start with magic number 0x%04x, got 0x%04x instead', BinaryTrajectory.MAGIC_NUMBER, magicNumber)

    if versionNumber < 1 or versionNumber > BinaryTrajectory.BINARY_TRAJECTORY_VERSION_NUMBER:
        raise ValueError('trajectory file has invalid version number %d', versionNumber)

    # get number of groups in configuration spec
    numGroups = struct.unpack_from('<H', data, offset=offset)[0]
    offset += struct.calcsize('<H')

    # read each group
    dof = 0
    groups = []
    for iGroup in range(numGroups):
        group = BinaryTrajectory.ConfigurationSpecificationGroup()
        group.name, offset = _ParseBinaryString(data, offset=offset)
        group.offset, group.dof = struct.unpack_from('<ii', data, offset=offset)
        offset += struct.calcsize('<ii')
        group.interpolation, offset = _ParseBinaryString(data, offset=offset)
        groups.append(group)
        dof = max(dof, group.offset + group.dof)

    # get number of data points
    numPoints = struct.unpack_from('<I', data, offset=offset)[0]
    offset += struct.calcsize('<I')

    # read all data points
    fmt = '<%dd' % numPoints
    points = struct.unpack_from(fmt, data, offset=offset)
    offset += struct.calcsize(fmt)

    # read description
    description, offset = _ParseBinaryString(data, offset=offset)

    # version 2 and above has readableInterfaces
    readableInterfaces = []
    if versionNumber >= 2:
        numReadableInterfaces = struct.unpack_from('<H', data, offset=offset)[0]
        offset += struct.calcsize('<H')

        for iReadableInterface in range(numReadableInterfaces):
            readableInterfaceId, offset = _ParseBinaryString(data, offset=offset)
            readableInterfaceData, offset = _ParseBinaryString(data, offset=offset)
            readableInterfaceType = None
            if versionNumber >= 3:
                readableInterfaceType, offset = _ParseBinaryString(data, offset=offset)
            readableInterfaces.append({
                'id': readableInterfaceId,
                'data': readableInterfaceData,
                'type': readableInterfaceType,
            })

    traj = BinaryTrajectory()
    traj.description = description
    traj.groups = groups
    traj.waypoints = numpy.array(points, dtype=numpy.float64).reshape(-1, dof)
    traj.readableInterfaces = readableInterfaces
    return traj, offset