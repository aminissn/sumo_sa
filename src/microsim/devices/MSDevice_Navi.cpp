/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2007-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_Navi.cpp
/// @author  Sasan Amini    
/// @date    2025-12-10
///
// A device that performs vehicle rerouting using historical travel times and logit model
/****************************************************************************/
#include <config.h>

#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSVehicleControl.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/WrappingCommand.h>
#include <utils/common/StringUtils.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/iodevices/OutputDevice_String.h>
#ifdef HAVE_FOX
#include <utils/foxtools/MFXWorkerThread.h>
#endif
#include "MSNaviEngine.h"
#include "MSDevice_Navi.h"

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Navi::insertOptions(OptionsCont& oc) {
    insertDefaultAssignmentOptions("navi", "Routing", oc);

    oc.doRegister("device.navi.period", new Option_String("0", "TIME"));
    oc.addDescription("device.navi.period", "Routing", TL("The period with which the vehicle shall be rerouted"));

    oc.doRegister("device.navi.pre-period", new Option_String("60", "TIME"));
    oc.addDescription("device.navi.pre-period", "Routing", TL("The rerouting period before depart"));

    oc.doRegister("device.navi.history-intervals", new Option_Integer(5));
    oc.addDescription("device.navi.history-intervals", "Routing", TL("Number of 60-second intervals to look back for average travel time"));

    oc.doRegister("device.navi.logit-theta", new Option_Float(1.0));
    oc.addDescription("device.navi.logit-theta", "Routing", TL("Theta parameter for logit model (higher = more deterministic)"));

    oc.doRegister("device.navi.max-alternatives", new Option_Integer(5));
    oc.addDescription("device.navi.max-alternatives", "Routing", TL("Maximum number of alternative routes to consider"));

    oc.doRegister("device.navi.output", new Option_FileName());
    oc.addDescription("device.navi.output", "Routing", TL("Save historical travel times to FILE"));

    oc.doRegister("device.navi.threshold.factor", new Option_Float(1));
    oc.addDescription("device.navi.threshold.factor", "Routing", TL("Only reroute if the new route is faster than the current route by the given factor"));

    oc.doRegister("device.navi.threshold.constant", new Option_String("0", "TIME"));
    oc.addDescription("device.navi.threshold.constant", "Routing", TL("Only reroute if the new route is faster than the current route by the given TIME"));

    oc.doRegister("device.navi.route-output", new Option_FileName());
    oc.addDescription("device.navi.route-output", "Routing", TL("Save computed routes to FILE"));

    oc.doRegister("device.navi.route-output.exit-times", new Option_Bool(false));
    oc.addDescription("device.navi.route-output.exit-times", "Routing", TL("Save exit times for each edge"));

    oc.doRegister("device.navi.route-output.costs", new Option_Bool(false));
    oc.addDescription("device.navi.route-output.costs", "Routing", TL("Save route costs"));

    oc.doRegister("device.navi.threads", new Option_Integer(0));
    oc.addDescription("device.navi.threads", "Routing", TL("The number of parallel execution threads used for routing (0 = use simulation threads)"));
}

bool
MSDevice_Navi::checkOptions(OptionsCont& oc) {
    bool ok = true;
    if (oc.getInt("device.navi.history-intervals") < 1) {
        WRITE_ERROR(TL("device.navi.history-intervals must be at least 1"));
        ok = false;
    }
    if (oc.getInt("device.navi.max-alternatives") < 1) {
        WRITE_ERROR(TL("device.navi.max-alternatives must be at least 1"));
        ok = false;
    }
    if (oc.getFloat("device.navi.logit-theta") < 0) {
        WRITE_ERROR(TL("device.navi.logit-theta must be non-negative"));
        ok = false;
    }
#ifndef HAVE_FOX
    if (oc.getInt("device.navi.threads") > 1) {
        WRITE_ERROR(TL("Parallel routing is only possible when compiled with Fox."));
        ok = false;
    }
#endif
    if (oc.isSet("device.navi.route-output")) {
        OutputDevice::createDeviceByOption("device.navi.route-output", "routes", "routes_file.xsd");
    }
    return ok;
}

void
MSDevice_Navi::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    const OptionsCont& oc = OptionsCont::getOptions();
    const bool equip = equippedByDefaultAssignmentOptions(oc, "navi", v, false);
    if (equip) {
        // Initialize navi engine on first device
        MSNaviEngine::init();
        
        // route computation is enabled
        const SUMOTime period = (equip || (
                                     oc.isDefault("device.navi.probability") &&
                                     v.getFloatParam("device.navi.probability") == oc.getFloat("device.navi.probability"))
                                 ? v.getTimeParam("device.navi.period") : 0);
        const SUMOTime prePeriod = MAX2((SUMOTime)0, v.getTimeParam("device.navi.pre-period"));
        
        // build the device
        into.push_back(new MSDevice_Navi(v, "navi_" + v.getID(), period, prePeriod));
    }
}

// ---------------------------------------------------------------------------
// MSDevice_Navi-methods
// ---------------------------------------------------------------------------
MSDevice_Navi::MSDevice_Navi(SUMOVehicle& holder, const std::string& id,
                             SUMOTime period, SUMOTime preInsertionPeriod) :
    MSVehicleDevice(holder, id),
    myPeriod(period),
    myPreInsertionPeriod(preInsertionPeriod),
    myLastRouting(-1),
    mySkipRouting(-1),
    myRerouteCommand(nullptr),
    myLastLaneEntryTime(-1),
    myRerouteAfterStop(false),
    myThresholdFactor(holder.getFloatParam("device.navi.threshold.factor", true, 1)),
    myThresholdTime(STEPS2TIME(holder.getTimeParam("device.navi.threshold.constant", true, 0))) {
    if (myPreInsertionPeriod > 0 || holder.getParameter().wasSet(VEHPARS_FORCE_REROUTE)) {
        // we do always a pre insertion reroute for trips
        myRerouteCommand = new WrappingCommand<MSDevice_Navi>(this, &MSDevice_Navi::preInsertionReroute);
        const SUMOTime execTime = holder.getParameter().depart;
        MSNet::getInstance()->getInsertionEvents()->addEvent(myRerouteCommand, execTime);
    }
}

MSDevice_Navi::~MSDevice_Navi() {
    // make the rerouting command invalid if there is one
    if (myRerouteCommand != nullptr) {
        myRerouteCommand->deschedule();
    }
}

bool
MSDevice_Navi::notifyEnter(SUMOTrafficObject& /*veh*/, MSMoveReminder::Notification reason, const MSLane* enteredLane) {
    if (reason == MSMoveReminder::NOTIFICATION_DEPARTED) {
        // build repetition trigger if routing shall be done more often
        rebuildRerouteCommand(SIMSTEP + myPeriod);
    }
    if (MSGlobals::gWeightsSeparateTurns > 0) {
        if (reason == MSMoveReminder::NOTIFICATION_JUNCTION) {
            const SUMOTime t = SIMSTEP;
            if (myLastLaneEntryTime >= 0 && enteredLane->isInternal()) {
                // record travel time on the previous edge
                MSNaviEngine::addEdgeTravelTime(enteredLane->getEdge(), t - myLastLaneEntryTime);
            }
            myLastLaneEntryTime = t;
        }
        return true;
    } else {
        return false;
    }
}

void
MSDevice_Navi::notifyStopEnded() {
    if (myRerouteAfterStop) {
        reroute(SIMSTEP);
        myRerouteAfterStop = false;
    }
}

void
MSDevice_Navi::rebuildRerouteCommand(SUMOTime start) {
    if (myRerouteCommand != nullptr) {
        myRerouteCommand->deschedule();
        myRerouteCommand = nullptr;
    }
    if (myPeriod > 0) {
        myRerouteCommand = new WrappingCommand<MSDevice_Navi>(this, &MSDevice_Navi::wrappedRerouteCommandExecute);
        // ensure stable sorting of events (for repeatable routing with randomness)
        myRerouteCommand->priority = (int)myHolder.getNumericalID();
        MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(myRerouteCommand, start);
    }
}

SUMOTime
MSDevice_Navi::preInsertionReroute(const SUMOTime currentTime) {
    if (mySkipRouting == currentTime) {
        return DELTA_T;
    }
    if (myPreInsertionPeriod == 0) {
        // the event will deschedule and destroy itself so it does not need to be stored
        myRerouteCommand = nullptr;
    }
    const MSEdge* source = *myHolder.getRoute().begin();
    const MSEdge* dest = myHolder.getRoute().getLastEdge();
    if (source->isTazConnector() && dest->isTazConnector()) {
        // For TAZ connectors, use standard routing
        try {
            std::string msg;
            if (myHolder.hasValidRouteStart(msg)) {
                reroute(currentTime, true);
            }
        } catch (ProcessError&) {
            myRerouteCommand = nullptr;
            throw;
        }
        return myPreInsertionPeriod;
    }
    try {
        std::string msg;
        if (myHolder.hasValidRouteStart(msg)) {
            reroute(currentTime, true);
        }
    } catch (ProcessError&) {
        myRerouteCommand = nullptr;
        throw;
    }
    // avoid repeated pre-insertion rerouting when the departure edge is fix
    if (myPreInsertionPeriod > 0 && !source->isTazConnector() && 
        myHolder.getParameter().departLaneProcedure != DepartLaneDefinition::BEST_FREE) {
        myRerouteCommand = nullptr;
        return 0;
    }
    return myPreInsertionPeriod;
}

SUMOTime
MSDevice_Navi::wrappedRerouteCommandExecute(SUMOTime currentTime) {
    if (myHolder.isStopped()) {
        myRerouteAfterStop = true;
    } else {
        reroute(currentTime);
    }
    return myPeriod;
}

void
MSDevice_Navi::reroute(const SUMOTime currentTime, const bool onInit) {
    MSNaviEngine::initEdgeWeights(myHolder.getVClass());
    // check whether we should reroute
    if (!onInit && (myLastRouting >= MSNaviEngine::getLastUpdate() || myLastRouting == currentTime)) {
        return;
    }
    
    myLastRouting = currentTime;
    
    // Use MSNaviEngine::reroute which handles parallelization via thread pool
    // This dispatches the routing task to a worker thread if available
    MSNaviEngine::reroute(myHolder, currentTime, "device.navi", onInit);
}

std::string
MSDevice_Navi::getParameter(const std::string& key) const {
    if (StringUtils::startsWith(key, "edge:")) {
        const std::string edgeID = key.substr(5);
        const MSEdge* edge = MSEdge::dictionary(edgeID);
        if (edge == nullptr) {
            throw InvalidArgument("Edge '" + edgeID + "' is invalid for parameter retrieval of '" + deviceName() + "'");
        }
        return toString(MSNaviEngine::getEffort(edge, &myHolder, 0));
    } else if (key == "period") {
        return time2string(myPeriod);
    } else if (key == "history-intervals") {
        return toString(MSNaviEngine::getHistoryIntervals());
    } else if (key == "logit-theta") {
        return toString(MSNaviEngine::getLogitTheta());
    } else if (key == "max-alternatives") {
        return toString(MSNaviEngine::getMaxAlternatives());
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}

void
MSDevice_Navi::setParameter(const std::string& key, const std::string& value) {
    if (key == "period") {
        double doubleValue;
        try {
            doubleValue = StringUtils::toDouble(value);
        } catch (NumberFormatException&) {
            throw InvalidArgument("Setting parameter 'period' requires a number for device of type '" + deviceName() + "'");
        }
        myPeriod = TIME2STEPS(doubleValue);
        // re-schedule routing command
        rebuildRerouteCommand(SIMSTEP + myPeriod);
    } else {
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}

void
MSDevice_Navi::saveState(OutputDevice& out) const {
    out.openTag(SUMO_TAG_DEVICE);
    out.writeAttr(SUMO_ATTR_ID, getID());
    std::vector<std::string> internals;
    internals.push_back(toString(myPeriod));
    internals.push_back(toString(myLastRouting));
    out.writeAttr(SUMO_ATTR_STATE, toString(internals));
    out.closeTag();
}

bool
MSDevice_Navi::sufficientSaving(double oldCost, double newCost) {
    if (newCost == 0) {
        return true;
    }
    return (oldCost / newCost > myThresholdFactor) && (oldCost - newCost > myThresholdTime);
}

void
MSDevice_Navi::loadState(const SUMOSAXAttributes& attrs) {
    std::istringstream bis(attrs.getString(SUMO_ATTR_STATE));
    bis >> myPeriod;
    bis >> myLastRouting;
    if (myHolder.hasDeparted()) {
        SUMOTime offset = myPeriod;
        if (myPeriod > 0) {
            offset = ((SIMSTEP - myHolder.getDeparture()) % myPeriod);
            if (offset != 0) {
                offset = myPeriod - offset;
            }
        }
        rebuildRerouteCommand(SIMSTEP + offset);
    }
}

void
MSDevice_Navi::writeRoute(const ConstMSRoutePtr& route, const SUMOTime currentTime, const double cost, const bool onInit) const {
    if (!OptionsCont::getOptions().isSet("device.navi.route-output")) {
        return;
    }
    
    OutputDevice& routeOut = OutputDevice::getDeviceByOption("device.navi.route-output");
    const OptionsCont& oc = OptionsCont::getOptions();
    const bool writeExitTimes = oc.getBool("device.navi.route-output.exit-times");
    const bool writeCosts = oc.getBool("device.navi.route-output.costs");
    
    routeOut.openTag(SUMO_TAG_ROUTE);
    routeOut.writeAttr(SUMO_ATTR_ID, myHolder.getID());
    routeOut.writeAttr(SUMO_ATTR_VEHICLE, myHolder.getID());
    
    if (writeCosts) {
        routeOut.writeAttr(SUMO_ATTR_COST, cost);
    }
    
    // Write reason and time
    routeOut.writeAttr("reason", onInit ? "init" : "reroute");
    routeOut.writeAttr(SUMO_ATTR_REPLACED_AT_TIME, time2string(currentTime));
    
    // Write edges
    OutputDevice_String edgesD;
    route->writeEdgeIDs(edgesD, 0, -1, false, myHolder.getVClass());
    std::string edgesS = edgesD.getString();
    if (!edgesS.empty()) {
        edgesS.pop_back(); // remove last ' '
    }
    routeOut.writeAttr(SUMO_ATTR_EDGES, edgesS);
    
    // Write exit times if requested
    if (writeExitTimes) {
        std::vector<std::string> exitTimes;
        double time = STEPS2TIME(currentTime);
        for (const MSEdge* e : route->getEdges()) {
            if (!e->isInternal() && !e->isTazConnector()) {
                time += MSNaviEngine::getEffort(e, &myHolder, TIME2STEPS(time));
                exitTimes.push_back(time2string(TIME2STEPS(time)));
            }
        }
        routeOut.writeAttr(SUMO_ATTR_EXITTIMES, exitTimes);
    }
    
    routeOut.closeTag();
}

/****************************************************************************/

