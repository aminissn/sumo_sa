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
    // Check if device.navi is enabled and multi-threading is requested
    // device.navi is NOT thread-safe due to shared k-shortest path routers
    const bool naviEnabled = oc.getFloat("device.navi.probability") > 0 || 
                             oc.isSet("device.navi.explicit") ||
                             oc.isSet("device.navi.deterministic");
    if (naviEnabled) {
        int numThreads = oc.getInt("threads");
        if (numThreads == 0) {
            numThreads = oc.getInt("device.rerouting.threads");
        }
        if (numThreads > 1) {
            WRITE_WARNING(TL("device.navi is not thread-safe. Using --threads 1 is recommended to avoid crashes."));
            WRITE_WARNING(TL("If you experience 'Error: vector' crashes, run with --threads 1"));
        }
    }
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
    try {
        if (reason == MSMoveReminder::NOTIFICATION_DEPARTED) {
            // build repetition trigger if routing shall be done more often
            rebuildRerouteCommand(SIMSTEP + myPeriod);
        }
        // In mesoscopic simulation (MESO), enteredLane is always nullptr
        // so we skip the lane-based travel time tracking
        if (!MSGlobals::gUseMesoSim && MSGlobals::gWeightsSeparateTurns > 0) {
            if (reason == MSMoveReminder::NOTIFICATION_JUNCTION) {
                const SUMOTime t = SIMSTEP;
                if (myLastLaneEntryTime >= 0 && enteredLane != nullptr && enteredLane->isInternal()) {
                    // record travel time on the previous edge
                    MSNaviEngine::addEdgeTravelTime(enteredLane->getEdge(), t - myLastLaneEntryTime);
                }
                myLastLaneEntryTime = t;
            }
            return true;
        } else {
            return false;
        }
    } catch (...) {
        // Ignore errors in notifyEnter to prevent simulation crash
        return false;
    }
}

void
MSDevice_Navi::notifyStopEnded() {
    try {
        if (myRerouteAfterStop) {
            reroute(SIMSTEP);
            myRerouteAfterStop = false;
        }
    } catch (...) {
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
    try {
        if (mySkipRouting == currentTime) {
            return DELTA_T;
        }
        if (myPreInsertionPeriod == 0) {
            // the event will deschedule and destroy itself so it does not need to be stored
            myRerouteCommand = nullptr;
        }
        
        // Safety check: ensure route is valid before accessing
        if (myHolder.getRoute().size() == 0) {
            return myPreInsertionPeriod;
        }
        
        const MSEdge* source = *myHolder.getRoute().begin();
        const MSEdge* dest = myHolder.getRoute().getLastEdge();
        
        if (source == nullptr || dest == nullptr) {
            return myPreInsertionPeriod;
        }
        
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
    } catch (const std::out_of_range& e) {
        WRITE_WARNING("device.navi preInsertionReroute out_of_range for '" + myHolder.getID() + "': " + e.what());
        return myPreInsertionPeriod;
    } catch (const std::exception& e) {
        WRITE_WARNING("device.navi preInsertionReroute error for '" + myHolder.getID() + "': " + e.what());
        return myPreInsertionPeriod;
    } catch (...) {
        WRITE_WARNING("device.navi preInsertionReroute unknown error for '" + myHolder.getID() + "'");
        return myPreInsertionPeriod;
    }
}

SUMOTime
MSDevice_Navi::wrappedRerouteCommandExecute(SUMOTime currentTime) {
    try {
        if (myHolder.isStopped()) {
            myRerouteAfterStop = true;
        } else {
            reroute(currentTime);
        }
    } catch (const std::exception& e) {
        WRITE_WARNING("device.navi wrappedRerouteCommandExecute error for '" + myHolder.getID() + "': " + e.what());
    } catch (...) {
        WRITE_WARNING("device.navi wrappedRerouteCommandExecute unknown error for '" + myHolder.getID() + "'");
    }
    return myPeriod;
}

void
MSDevice_Navi::reroute(const SUMOTime currentTime, const bool onInit) {
    try {
        // Safety check: verify holder is in valid state
        if (!myHolder.isOnRoad() && !onInit) {
            return;  // Vehicle not on road, skip rerouting
        }
        
        MSNaviEngine::initEdgeWeights(myHolder.getVClass());
    } catch (const std::exception& e) {
        WRITE_WARNING("device.navi initEdgeWeights error for '" + myHolder.getID() + "': " + e.what());
        return;
    } catch (...) {
        WRITE_WARNING("device.navi initEdgeWeights unknown error for '" + myHolder.getID() + "'");
        return;
    }
    
    try {
        // check whether we should reroute
        if (!onInit && (myLastRouting >= MSNaviEngine::getLastUpdate() || myLastRouting == currentTime)) {
            return;
        }
        
        myLastRouting = currentTime;
        
        // Safety check: verify route is valid
        try {
            if (myHolder.getRoute().size() == 0) {
                return;
            }
        } catch (...) {
            return;
        }
        
        // Use MSNaviEngine::reroute which handles parallelization via thread pool
        // This dispatches the routing task to a worker thread if available
        MSNaviEngine::RerouteResult result = MSNaviEngine::reroute(myHolder, currentTime, "device.navi", onInit);
        
        // Write route output (includes all alternatives even if no rerouting happened)
        if (!result.alternatives.empty()) {
            writeRoute(result, currentTime, onInit);
        }
    } catch (const std::out_of_range& e) {
        WRITE_WARNING("device.navi reroute out_of_range for '" + myHolder.getID() + "': " + e.what());
    } catch (const std::exception& e) {
        WRITE_WARNING("device.navi reroute error for '" + myHolder.getID() + "': " + e.what());
    } catch (...) {
        WRITE_WARNING("device.navi reroute unknown error for '" + myHolder.getID() + "'");
    }
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
MSDevice_Navi::writeRoute(const MSNaviEngine::RerouteResult& result, const SUMOTime currentTime, const bool onInit) const {
    try {
        if (!OptionsCont::getOptions().isSet("device.navi.route-output")) {
            return;
        }
        
        OutputDevice& routeOut = OutputDevice::getDeviceByOption("device.navi.route-output");
        const OptionsCont& oc = OptionsCont::getOptions();
        const bool writeExitTimes = oc.getBool("device.navi.route-output.exit-times");
        const bool writeCosts = oc.getBool("device.navi.route-output.costs");
        
        // Open routeChoice element to group all alternatives for this vehicle at this time
        routeOut.openTag("routeChoice");
        routeOut.writeAttr("vehicle", myHolder.getID());
        routeOut.writeAttr("time", time2string(currentTime));
        routeOut.writeAttr("reason", onInit ? "device.navi:init" : "device.navi:reroute");
        routeOut.writeAttr("rerouted", result.success ? "true" : "false");
        
        // Write the edge where vehicle was when route was computed
        // In MESO, getLane() always returns nullptr, so we use getEdge() instead
        if (myHolder.hasDeparted()) {
            try {
                const MSEdge* currentEdge = myHolder.getEdge();
                if (currentEdge != nullptr) {
                    routeOut.writeAttr("replacedOnEdge", currentEdge->getID());
                }
            } catch (...) {
                // Ignore edge access errors
            }
        }
        
        // Write old route cost if available (write even if 0 for onInit cases)
        if (writeCosts) {
            routeOut.writeAttr("oldCost", result.oldCost);
        }
        
        // Write each alternative route
        int altIndex = 0;
        for (size_t i = 0; i < result.alternatives.size(); ++i) {
            const auto& alt = result.alternatives[i];
            if (alt.route == nullptr) {
                continue;
            }
            
            // Safety check on route size
            try {
                if (alt.route->size() == 0) {
                    continue;
                }
            } catch (...) {
                continue;
            }
            
            routeOut.openTag(SUMO_TAG_ROUTE);
            routeOut.writeAttr(SUMO_ATTR_ID, myHolder.getID() + "_" + time2string(currentTime) + "_alt" + toString(altIndex));
            
            // Write cost and probability
            if (writeCosts) {
                // Use cost from alternative (calculated during routing), not from route object
                // The route object may have default cost of -1 if not set
                double routeCost = alt.cost;
                // If alternative cost is invalid, try to get it from route object as fallback
                if (routeCost < 0 && alt.route != nullptr) {
                    routeCost = alt.route->getCosts();
                }
                routeOut.writeAttr(SUMO_ATTR_COST, routeCost);
                
                // Calculate and write savings (oldCost - newCost)
                // Savings can be negative (onInit case when oldCost is 0)
                double savings = result.oldCost - routeCost;
                routeOut.writeAttr(SUMO_ATTR_SAVINGS, savings);
            }
            routeOut.writeAttr(SUMO_ATTR_PROB, alt.probability);
            routeOut.writeAttr("selected", alt.selected ? "true" : "false");
            
            // Write edges with safety check
            try {
                OutputDevice_String edgesD;
                alt.route->writeEdgeIDs(edgesD, 0, -1, false, myHolder.getVClass());
                std::string edgesS = edgesD.getString();
                if (!edgesS.empty()) {
                    edgesS.pop_back(); // remove last ' '
                }
                routeOut.writeAttr(SUMO_ATTR_EDGES, edgesS);
            } catch (...) {
                routeOut.writeAttr(SUMO_ATTR_EDGES, "");
            }
            
            // Write exit times if requested (only for selected route to reduce output size)
            if (writeExitTimes && alt.selected) {
                try {
                    std::vector<std::string> exitTimes;
                    double time = STEPS2TIME(currentTime);
                    const ConstMSEdgeVector& edges = alt.route->getEdges();
                    for (size_t j = 0; j < edges.size(); ++j) {
                        const MSEdge* e = edges[j];
                        if (e != nullptr && !e->isInternal() && !e->isTazConnector()) {
                            time += MSNaviEngine::getEffort(e, &myHolder, TIME2STEPS(time));
                            exitTimes.push_back(time2string(TIME2STEPS(time)));
                        }
                    }
                    routeOut.writeAttr(SUMO_ATTR_EXITTIMES, exitTimes);
                } catch (...) {
                    // Skip exit times on error
                }
            }
            
            routeOut.closeTag();
            altIndex++;
        }
        
        routeOut.closeTag();  // close routeChoice
    } catch (const std::exception& e) {
        WRITE_WARNING("device.navi writeRoute error for '" + myHolder.getID() + "': " + e.what());
    } catch (...) {
        WRITE_WARNING("device.navi writeRoute unknown error for '" + myHolder.getID() + "'");
    }
}

/****************************************************************************/

