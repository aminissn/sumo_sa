#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2009-2025 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    test.py
# @author  Pablo Alvarez Lopez
# @date    2016-11-25

# import common functions for netedit tests
import os
import sys

sys.path.append(os.path.join(os.environ.get("SUMO_HOME", "."), "tools"))
import neteditTestFunctions as netedit  # noqa

# Open netedit
neteditProcess, referencePosition = netedit.setupAndStart()

# go to additional mode
netedit.changeMode("additional")

# select E1
netedit.changeElement("additionalFrame", "inductionLoop")

# create E1
netedit.leftClick(referencePosition, netedit.positions.elements.edge0)

# Change to delete
netedit.changeMode("delete")

# delete created E1
netedit.leftClick(referencePosition, netedit.positions.elements.edge0)

# delete loaded E1
netedit.leftClick(referencePosition, netedit.positions.elements.edge4)

# Check undo
netedit.undo(referencePosition, 2)

# Change to delete
netedit.changeMode("delete")

# try to delete lane with the second loaded E1 (doesn"t allowed)
netedit.leftClick(referencePosition, netedit.positions.elements.edge0Ped)

# wait warning
netedit.waitDeleteWarning()

# disable "Automatically delete additionals"
netedit.protectElements()

# try to delete lane with the second loaded E1 (doesn"t allowed)
netedit.leftClick(referencePosition, netedit.positions.elements.edge0Ped)

# check redo
netedit.undo(referencePosition, 4)
netedit.redo(referencePosition, 4)

# save netedit config
netedit.saveExistentShortcut("neteditConfig")

# quit netedit
netedit.quit(neteditProcess)
