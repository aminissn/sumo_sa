/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2026 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSODRouteExport.cpp
/// @author  aminissn
/// @date    2026-09-14
///
// Aggregated route flows for each origin-destination (TAZ) pair
/****************************************************************************/
#include <config.h>

#include <algorithm>
#include <microsim/MSEdge.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/MSRoute.h>
#include <microsim/MSVehicleControl.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <utils/vehicle/SUMOVehicle.h>
#include "MSODRouteExport.h"


// ===========================================================================
// static member definitions
// ===========================================================================
bool MSODRouteExport::myActive = false;
bool MSODRouteExport::myWriteUnfinished = false;
SUMOTime MSODRouteExport::myPeriod = -1;
SUMOTime MSODRouteExport::myIntervalBegin = 0;
bool MSODRouteExport::myWroteInterval = false;
int MSODRouteExport::myUseTaz = -1;
std::map<const MSEdge*, std::string> MSODRouteExport::myOrigins;
std::map<const MSEdge*, std::string> MSODRouteExport::myDestinations;
std::map<std::pair<std::string, std::string>, MSODRouteExport::ODCount> MSODRouteExport::myCounts;


// ===========================================================================
// method definitions
// ===========================================================================
void
MSODRouteExport::init() {
    const OptionsCont& oc = OptionsCont::getOptions();
    // depth 3: interval > tazRelation > route
    myActive = OutputDevice::createDeviceByOption("od-route-output", "odRoutes", "", 3);
    if (myActive) {
        myWriteUnfinished = oc.getBool("od-route-output.write-unfinished");
        myPeriod = string2time(oc.getString("od-route-output.period"));
        myIntervalBegin = string2time(oc.getString("begin"));
        myWroteInterval = false;
        // TAZ are loaded from additional files after the streams have been built, so decide lazily
        myUseTaz = oc.getBool("od-route-output.edges") ? 0 : -1;
        myOrigins.clear();
        myDestinations.clear();
        myCounts.clear();
    }
}


const std::string&
MSODRouteExport::getZone(const MSEdge* edge, const bool origin) {
    std::map<const MSEdge*, std::string>& cache = origin ? myOrigins : myDestinations;
    auto it = cache.find(edge);
    if (it != cache.end()) {
        return it->second;
    }
    if (myUseTaz < 0) {
        myUseTaz = 0;
        for (const MSEdge* const e : MSEdge::getAllEdges()) {
            if (e->isTazConnector()) {
                myUseTaz = 1;
                break;
            }
        }
    }
    std::string zone = edge->getID();
    if (myUseTaz == 1) {
        // TAZ sources are predecessors of their edges, TAZ sinks are successors
        const MSEdgeVector& candidates = origin ? edge->getPredecessors() : edge->getSuccessors();
        std::string taz;
        for (const MSEdge* const c : candidates) {
            if (c->isTazConnector()) {
                std::string id = c->getParameter("taz");
                if (id == "") {
                    // junction taz connectors do not carry the parameter
                    const std::string suffix = origin ? "-source" : "-sink";
                    id = c->getID().substr(0, c->getID().size() - suffix.size());
                }
                // an edge may belong to several TAZ, use the first in alphabetical order for determinism
                if (taz == "" || id < taz) {
                    taz = id;
                }
            }
        }
        if (taz != "") {
            zone = taz;
        }
    }
    return cache.emplace(edge, zone).first->second;
}


void
MSODRouteExport::addVehicle(const SUMOVehicle& veh, const bool arrived) {
    const MSRoute& route = veh.getRoute();
    const ConstMSEdgeVector& edges = route.getEdges();
    if (edges.empty()) {
        return;
    }
    const std::string& from = getZone(edges.front(), true);
    const std::string& to = getZone(edges.back(), false);
    ODCount& od = myCounts[std::make_pair(from, to)];
    od.count++;
    std::vector<int> key;
    key.reserve(edges.size());
    for (const MSEdge* const e : edges) {
        key.push_back(e->getNumericalID());
    }
    RouteCount& rc = od.routes[key];
    if (rc.count == 0) {
        rc.edges = edges;
        const bool includeInternalLengths = MSGlobals::gUsingInternalLanes && MSNet::getInstance()->hasInternalLinks();
        rc.length = route.getDistanceBetween(0., edges.back()->getLength(), route.begin(), route.end() - 1, includeInternalLengths);
    }
    rc.count++;
    if (arrived) {
        rc.arrived++;
        rc.travelTime += STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() - veh.getDeparture());
    }
}


void
MSODRouteExport::write(const SUMOTime step) {
    if (myPeriod > 0 && step + DELTA_T >= myIntervalBegin + myPeriod) {
        const SUMOTime end = myIntervalBegin + myPeriod;
        if (!myCounts.empty()) {
            writeInterval(myIntervalBegin, end);
        }
        myIntervalBegin = end;
    }
}


void
MSODRouteExport::finish(const SUMOTime step) {
    if (myWriteUnfinished) {
        const MSVehicleControl& vc = MSNet::getInstance()->getVehicleControl();
        for (MSVehicleControl::constVehIt it = vc.loadedVehBegin(); it != vc.loadedVehEnd(); ++it) {
            const SUMOVehicle* const veh = it->second;
            if (veh->hasDeparted() && !veh->hasArrived()) {
                addVehicle(*veh, false);
            }
        }
    }
    if (!myCounts.empty() || !myWroteInterval) {
        writeInterval(myIntervalBegin, step);
    }
}


void
MSODRouteExport::writeInterval(const SUMOTime begin, const SUMOTime end) {
    OutputDevice& od = OutputDevice::getDeviceByOption("od-route-output");
    od.openTag(SUMO_TAG_INTERVAL);
    od.writeTime(SUMO_ATTR_BEGIN, begin);
    od.writeTime(SUMO_ATTR_END, end);
    for (const auto& odItem : myCounts) {
        const ODCount& odCount = odItem.second;
        od.openTag(myUseTaz == 1 ? SUMO_TAG_TAZREL : SUMO_TAG_EDGEREL);
        od.writeAttr(SUMO_ATTR_FROM, odItem.first.first);
        od.writeAttr(SUMO_ATTR_TO, odItem.first.second);
        od.writeAttr(SUMO_ATTR_COUNT, odCount.count);
        // most frequently used route first, edge sequence as tie breaker
        std::vector<const RouteCount*> sorted;
        sorted.reserve(odCount.routes.size());
        for (const auto& routeItem : odCount.routes) {
            sorted.push_back(&routeItem.second);
        }
        std::stable_sort(sorted.begin(), sorted.end(), [](const RouteCount* a, const RouteCount* b) {
            return a->count > b->count;
        });
        int index = 0;
        for (const RouteCount* const rc : sorted) {
            od.openTag(SUMO_TAG_ROUTE);
            od.writeAttr(SUMO_ATTR_ID, odItem.first.first + "_" + odItem.first.second + "_" + toString(index++));
            od.writeAttr(SUMO_ATTR_EDGES, toString(rc->edges));
            od.writeAttr(SUMO_ATTR_COUNT, rc->count);
            od.writeAttr(SUMO_ATTR_PROB, (double)rc->count / (double)odCount.count);
            od.writeAttr(SUMO_ATTR_LENGTH, rc->length);
            // mean travel time of the arrived vehicles, -1 if none arrived (write-unfinished)
            od.writeAttr(SUMO_ATTR_TRAVELTIME, rc->arrived > 0 ? rc->travelTime / (double)rc->arrived : -1.);
            od.closeTag();
        }
        od.closeTag();
    }
    od.closeTag();
    myCounts.clear();
    myWroteInterval = true;
}


void
MSODRouteExport::cleanup() {
    myActive = false;
    myWriteUnfinished = false;
    myPeriod = -1;
    myIntervalBegin = 0;
    myWroteInterval = false;
    myUseTaz = -1;
    myOrigins.clear();
    myDestinations.clear();
    myCounts.clear();
}


/****************************************************************************/
