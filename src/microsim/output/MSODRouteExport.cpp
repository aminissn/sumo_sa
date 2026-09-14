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
#include <fstream>
#include <microsim/MSEdge.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/MSRoute.h>
#include <microsim/MSVehicleControl.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringUtils.h>
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
bool MSODRouteExport::myIntermediate = false;
bool MSODRouteExport::myHaveEdgeFilter = false;
std::set<const MSEdge*> MSODRouteExport::myEdgeFilter;
std::map<const MSEdge*, MSODRouteExport::Zone> MSODRouteExport::myOrigins;
std::map<const MSEdge*, MSODRouteExport::Zone> MSODRouteExport::myDestinations;
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
        myIntermediate = oc.getBool("od-route-output.intermediate");
        // TAZ and edges are loaded after the streams have been built, so decide lazily
        myUseTaz = -1;
        myHaveEdgeFilter = false;
        myEdgeFilter.clear();
        myOrigins.clear();
        myDestinations.clear();
        myCounts.clear();
    }
}


void
MSODRouteExport::initOnce() {
    const OptionsCont& oc = OptionsCont::getOptions();
    myUseTaz = 0;
    if (!oc.getBool("od-route-output.edges")) {
        for (const MSEdge* const e : MSEdge::getAllEdges()) {
            if (e->isTazConnector()) {
                myUseTaz = 1;
                break;
            }
        }
    }
    if (oc.isSet("od-route-output.filter-edges.input-file")) {
        if (myUseTaz == 1) {
            WRITE_WARNING(TL("Option od-route-output.filter-edges.input-file is ignored when aggregating by TAZ."));
        } else {
            const std::string file = oc.getString("od-route-output.filter-edges.input-file");
            std::ifstream strm(file.c_str());
            if (!strm.good()) {
                throw ProcessError(TLF("Could not load names of edges for filtering od-route-output from '%'.", file));
            }
            while (strm.good()) {
                std::string name;
                strm >> name;
                // maybe we're loading an edge-selection
                if (StringUtils::startsWith(name, "edge:")) {
                    name = name.substr(5);
                }
                const MSEdge* const edge = MSEdge::dictionary(name);
                if (edge != nullptr) {
                    myEdgeFilter.insert(edge);
                } else if (name != "") {
                    WRITE_WARNINGF(TL("Unknown edge '%' in od-route-output.filter-edges.input-file."), name);
                }
            }
            myHaveEdgeFilter = true;
        }
    }
    if (myIntermediate && myUseTaz == 0 && !myHaveEdgeFilter) {
        WRITE_WARNING(TL("Option od-route-output.intermediate counts all edge pairs of every route. Consider option od-route-output.filter-edges.input-file to limit the output size."));
    }
}


const MSODRouteExport::Zone&
MSODRouteExport::getZone(const MSEdge* edge, const bool origin) {
    std::map<const MSEdge*, Zone>& cache = origin ? myOrigins : myDestinations;
    auto it = cache.find(edge);
    if (it != cache.end()) {
        return it->second;
    }
    Zone zone;
    zone.id = edge->getID();
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
            zone.id = taz;
            zone.isTaz = true;
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
    if (myUseTaz < 0) {
        initOnce();
    }
    const int last = (int)edges.size() - 1;
    // the travel time is only known for the complete trip
    const double travelTime = arrived ? STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() - veh.getDeparture()) : -1.;
    if (!myIntermediate) {
        if (myHaveEdgeFilter && (myEdgeFilter.count(edges.front()) == 0 || myEdgeFilter.count(edges.back()) == 0)) {
            return;
        }
        addSubRoute(route, 0, last, getZone(edges.front(), true).id, getZone(edges.back(), false).id, travelTime);
    } else if (myUseTaz == 1) {
        // first position of every origin zone (source edges) and last position of every destination zone (sink edges)
        // edges without TAZ only act as departure and arrival edge
        std::map<std::string, int> firstOrigin;
        std::map<std::string, int> lastDestination;
        for (int p = 0; p <= last; p++) {
            const Zone& origin = getZone(edges[p], true);
            if ((origin.isTaz || p == 0) && firstOrigin.count(origin.id) == 0) {
                firstOrigin[origin.id] = p;
            }
            const Zone& destination = getZone(edges[p], false);
            if (destination.isTaz || p == last) {
                lastDestination[destination.id] = p;
            }
        }
        for (const auto& o : firstOrigin) {
            for (const auto& d : lastDestination) {
                const bool complete = o.second == 0 && d.second == last;
                // intra-zonal sub-routes are only of interest for the complete trip
                if (o.second <= d.second && (o.first != d.first || complete)) {
                    addSubRoute(route, o.second, d.second, o.first, d.first, complete ? travelTime : -1.);
                }
            }
        }
    } else {
        for (int i = 0; i < last; i++) {
            if (myHaveEdgeFilter && myEdgeFilter.count(edges[i]) == 0) {
                continue;
            }
            for (int j = i + 1; j <= last; j++) {
                if (myHaveEdgeFilter && myEdgeFilter.count(edges[j]) == 0) {
                    continue;
                }
                addSubRoute(route, i, j, edges[i]->getID(), edges[j]->getID(), (i == 0 && j == last) ? travelTime : -1.);
            }
        }
        if (last == 0 && (!myHaveEdgeFilter || myEdgeFilter.count(edges[0]) > 0)) {
            // single edge route
            addSubRoute(route, 0, 0, edges[0]->getID(), edges[0]->getID(), travelTime);
        }
    }
}


void
MSODRouteExport::addSubRoute(const MSRoute& route, const int i, const int j, const std::string& from, const std::string& to, const double travelTime) {
    const ConstMSEdgeVector& edges = route.getEdges();
    ODCount& od = myCounts[std::make_pair(from, to)];
    od.count++;
    std::vector<int> key;
    key.reserve(j - i + 1);
    for (int p = i; p <= j; p++) {
        key.push_back(edges[p]->getNumericalID());
    }
    RouteCount& rc = od.routes[key];
    if (rc.count == 0) {
        rc.edges.assign(edges.begin() + i, edges.begin() + j + 1);
        const bool includeInternalLengths = MSGlobals::gUsingInternalLanes && MSNet::getInstance()->hasInternalLinks();
        rc.length = route.getDistanceBetween(0., edges[j]->getLength(), route.begin() + i, route.begin() + j, includeInternalLengths);
    }
    rc.count++;
    if (travelTime >= 0) {
        rc.arrived++;
        rc.travelTime += travelTime;
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
    myIntermediate = false;
    myHaveEdgeFilter = false;
    myEdgeFilter.clear();
    myOrigins.clear();
    myDestinations.clear();
    myCounts.clear();
}


/****************************************************************************/
