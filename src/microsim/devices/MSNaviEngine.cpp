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
/// @file    MSNaviEngine.cpp
/// @author  Sasan Amini    
/// @date    2025-12-10
///
// Engine for device.navi that manages historical travel times and route finding
/****************************************************************************/
#include <config.h>

#include "MSNaviEngine.h"
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSVehicleControl.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/WrappingCommand.h>
#include <utils/common/StaticCommand.h>
#include <utils/common/StringUtils.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StdDefs.h>
#include <utils/common/RandHelper.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include <utils/router/DijkstraRouter.h>
#include <utils/router/AStarRouter.h>
#include <utils/router/CHRouter.h>
#include <utils/router/CHRouterWrapper.h>
#include <utils/router/RailwayRouter.h>
#include <utils/vehicle/SUMOVehicleParserHelper.h>
#include <utils/iodevices/OutputDevice.h>
#include <microsim/MSRoute.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSRouterDefs.h>
#include <algorithm>
#include <cmath>
#include <random>
#include <limits>
#include <map>
#include <set>
#ifdef HAVE_FOX
#include <utils/foxtools/fxheader.h>
#include <FXThread.h>
#endif

// ===========================================================================
// static member variables
// ===========================================================================
const SUMOTime MSNaviEngine::UPDATE_INTERVAL = TIME2STEPS(60); // 60 seconds
Command* MSNaviEngine::myTravelTimeUpdateCommand = nullptr;
int MSNaviEngine::myHistoryIntervals = 5;
double MSNaviEngine::myLogitTheta = 1.0;
int MSNaviEngine::myMaxAlternatives = 5;
int MSNaviEngine::myCurrentIntervalIndex = 0;
std::map<const MSEdge*, std::deque<MSNaviEngine::IntervalData> > MSNaviEngine::myHistoricalTravelTimes;
std::map<const MSEdge*, MSNaviEngine::IntervalData> MSNaviEngine::myCurrentIntervalData;
SUMOTime MSNaviEngine::myLastUpdate = -1;
MSRouterProvider* MSNaviEngine::myRouterProvider = nullptr;
SUMOAbstractRouter<MSEdge, SUMOVehicle>* MSNaviEngine::myKShortestRouter = nullptr;
bool MSNaviEngine::myInitialized = false;
#ifdef HAVE_FOX
FXMutex MSNaviEngine::myRouteCacheMutex;
FXMutex MSNaviEngine::myDataMutex;
std::map<std::pair<const MSEdge*, const MSEdge*>, ConstMSRoutePtr> MSNaviEngine::myCachedRoutes;
#endif

// ===========================================================================
// helper functions for k-shortest paths
// ===========================================================================
// Thread-local storage for penalties during k-shortest path finding
static thread_local std::map<const MSEdge*, double>* g_penalties = nullptr;

// Static function for penalized effort calculation
static double penalizedEffortFunc(const MSEdge* e, const SUMOVehicle* v, double t) {
    double baseEffort = MSNaviEngine::getEffort(e, v, t);
    if (g_penalties != nullptr) {
        auto it = g_penalties->find(e);
        if (it != g_penalties->end()) {
            return baseEffort + it->second;
        }
    }
    return baseEffort;
}

// ===========================================================================
// method definitions
// ===========================================================================
void
MSNaviEngine::init() {
    if (myInitialized) {
        return;
    }
    const OptionsCont& oc = OptionsCont::getOptions();
    myHistoryIntervals = oc.getInt("device.navi.history-intervals");
    myLogitTheta = oc.getFloat("device.navi.logit-theta");
    myMaxAlternatives = oc.getInt("device.navi.max-alternatives");
    
    if (myHistoryIntervals < 1) {
        WRITE_ERROR(TL("device.navi.history-intervals must be at least 1"));
        myHistoryIntervals = 1;
    }
    if (myMaxAlternatives < 1) {
        WRITE_ERROR(TL("device.navi.max-alternatives must be at least 1"));
        myMaxAlternatives = 1;
    }
    
    // Initialize edge weights
    initEdgeWeights(SVC_PASSENGER);
    
    // Schedule travel time update every 60 seconds
    myTravelTimeUpdateCommand = new StaticCommand<MSNaviEngine>(&MSNaviEngine::updateTravelTimes);
    MSNet::getInstance()->getEndOfTimestepEvents()->addEvent(myTravelTimeUpdateCommand, UPDATE_INTERVAL);
    myLastUpdate = SIMSTEP;
    
    // Create output device if requested
    OutputDevice::createDeviceByOption("device.navi.output", "weights", "meandata_file.xsd");
    
    myInitialized = true;
}

void
MSNaviEngine::initEdgeWeights(SUMOVehicleClass svc) {
#ifdef HAVE_FOX
    FXMutexLock lock(myDataMutex);
#endif
    // Initialize historical travel times for all edges
    const MSEdgeVector& edges = MSNet::getInstance()->getEdgeControl().getEdges();
    for (const MSEdge* edge : edges) {
        if (edge == nullptr) {
            continue;
        }
        if (myHistoricalTravelTimes.find(edge) == myHistoricalTravelTimes.end()) {
            // Initialize with empty deque
            myHistoricalTravelTimes[edge] = std::deque<IntervalData>();
            // Fill with initial data based on edge's free-flow travel time
            // Protect against division by zero for special edges
            const double speedLimit = edge->getSpeedLimit();
            const double length = edge->getLength();
            double freeFlowTT = 1.0; // Default minimum
            if (speedLimit > 0 && length > 0) {
                freeFlowTT = length / speedLimit;
            }
            IntervalData initialData;
            initialData.totalTravelTime = freeFlowTT;
            initialData.vehicleCount = 1;
            for (int i = 0; i < myHistoryIntervals; ++i) {
                myHistoricalTravelTimes[edge].push_back(initialData);
            }
        }
        myCurrentIntervalData[edge] = IntervalData();
    }
}

SUMOTime
MSNaviEngine::updateTravelTimes(SUMOTime currentTime) {
    try {
#ifdef HAVE_FOX
        FXMutexLock lock(myDataMutex);
#endif
        // Save current interval data to history
        for (auto& pair : myCurrentIntervalData) {
            const MSEdge* edge = pair.first;
            if (edge == nullptr) {
                continue;
            }
            IntervalData& data = pair.second;
            
            // Calculate average for this interval
            double avgTT = data.getAverage();
            if (avgTT == 0.0) {
                // No data, use free-flow travel time
                // Protect against division by zero
                const double speedLimit = edge->getSpeedLimit();
                const double length = edge->getLength();
                if (speedLimit > 0 && length > 0) {
                    avgTT = length / speedLimit;
                } else {
                    avgTT = 1.0; // Minimum travel time
                }
            }
            
            // Add to history - with safety checks
            auto histIt = myHistoricalTravelTimes.find(edge);
            if (histIt != myHistoricalTravelTimes.end()) {
                std::deque<IntervalData>& history = histIt->second;
                if ((int)history.size() >= myHistoryIntervals) {
                    history.pop_front();
                }
                IntervalData intervalData;
                intervalData.totalTravelTime = avgTT;
                intervalData.vehicleCount = 1;
                history.push_back(intervalData);
            }
            
            // Reset current interval
            data = IntervalData();
        }
        
        myLastUpdate = currentTime;
        
        // Write output if requested
        if (OptionsCont::getOptions().isSet("device.navi.output")) {
            OutputDevice& dev = OutputDevice::getDeviceByOption("device.navi.output");
            dev.openTag("interval");
            dev.writeAttr("begin", time2string(currentTime - UPDATE_INTERVAL));
            dev.writeAttr("end", time2string(currentTime));
            for (const auto& pair : myHistoricalTravelTimes) {
                const MSEdge* edge = pair.first;
                if (edge == nullptr) {
                    continue;
                }
                const std::deque<IntervalData>& history = pair.second;
                if (!history.empty()) {
                    double avgTT = getAverageTravelTime(edge, myHistoryIntervals);
                    dev.openTag("edge");
                    dev.writeAttr("id", edge->getID());
                    dev.writeAttr("traveltime", avgTT);
                    dev.closeTag();
                }
            }
            dev.closeTag();
        }
    } catch (const std::exception& e) {
        WRITE_WARNING("updateTravelTimes error: " + std::string(e.what()));
    } catch (...) {
        WRITE_WARNING("updateTravelTimes unknown error");
    }
    
    return UPDATE_INTERVAL;
}

double
MSNaviEngine::getEffort(const MSEdge* const e, const SUMOVehicle* const v, double t) {
    // Return average travel time over historical intervals
    return getAverageTravelTime(e, myHistoryIntervals);
}

double
MSNaviEngine::getAverageTravelTime(const MSEdge* edge, int intervals) {
    // Safety check
    if (edge == nullptr) {
        return 1.0;
    }
    
    // Helper to get safe free-flow travel time
    auto getFreeFlowTT = [](const MSEdge* e) -> double {
        const double speedLimit = e->getSpeedLimit();
        const double length = e->getLength();
        if (speedLimit > 0 && length > 0) {
            return length / speedLimit;
        }
        return 1.0; // Minimum default
    };
    
    // Note: No mutex here - this function is called from within already-locked sections
    // (findKShortestPaths, updateTravelTimes) or from getEffort which is called during routing
    // The caller is responsible for holding the lock if needed
    
    auto it = myHistoricalTravelTimes.find(edge);
    if (it == myHistoricalTravelTimes.end()) {
        // No history, return free-flow travel time
        return getFreeFlowTT(edge);
    }
    
    const std::deque<IntervalData>& history = it->second;
    if (history.empty()) {
        return getFreeFlowTT(edge);
    }
    
    // Calculate average over the last 'intervals' intervals
    double sum = 0.0;
    int count = 0;
    int lookBack = MIN2(intervals, (int)history.size());
    int startIdx = (int)history.size() - lookBack;
    for (int i = startIdx; i < (int)history.size(); ++i) {
        sum += history[i].getAverage();
        count++;
    }
    
    if (count > 0) {
        return sum / count;
    }
    return edge->getLength() / edge->getSpeedLimit();
}

ConstMSRoutePtr
MSNaviEngine::findAndSelectRoute(SUMOVehicle& vehicle, const SUMOTime currentTime,
                                  double& newCost, const bool onInit,
                                  std::vector<AlternativeInfo>* allAlternatives) {
    // Cache route reference to avoid race conditions
    ConstMSRoutePtr currentRoute;
    try {
        currentRoute = vehicle.getRoutePtr();
        if (currentRoute == nullptr || currentRoute->size() == 0) {
            return nullptr;
        }
    } catch (const std::exception& e) {
        WRITE_WARNING("findAndSelectRoute: Failed to get route for vehicle '" + vehicle.getID() + "': " + e.what());
        return nullptr;
    } catch (...) {
        WRITE_WARNING("findAndSelectRoute: Failed to get route for vehicle '" + vehicle.getID() + "'");
        return nullptr;
    }
    
    try {
    if (myRouterProvider == nullptr) {
        // Initialize router similar to MSRoutingEngine
        OptionsCont& oc = OptionsCont::getOptions();
        const std::string routingAlgorithm = oc.getString("routing-algorithm");
        const bool hasPermissions = MSNet::getInstance()->hasPermissions();
        
        SUMOAbstractRouter<MSEdge, SUMOVehicle>::Operation effortFunc = &MSNaviEngine::getEffort;
        
        SUMOAbstractRouter<MSEdge, SUMOVehicle>* router = nullptr;
        if (routingAlgorithm == "dijkstra") {
            router = new DijkstraRouter<MSEdge, SUMOVehicle>(MSEdge::getAllEdges(), true, effortFunc, nullptr, false, nullptr, true);
        } else if (routingAlgorithm == "astar") {
            typedef AStarRouter<MSEdge, SUMOVehicle, MSMapMatcher> AStar;
            router = new AStar(MSEdge::getAllEdges(), true, effortFunc, nullptr, true);
        } else if (routingAlgorithm == "CH" && !hasPermissions) {
            router = new CHRouter<MSEdge, SUMOVehicle>(
                MSEdge::getAllEdges(), true, effortFunc, vehicle.getVClass(), SUMOTime_MAX, true, false);
        } else if (routingAlgorithm == "CHWrapper" || routingAlgorithm == "CH") {
            // Use device.navi.threads if set, otherwise fall back to device.rerouting.threads or 0
            int numThreads = oc.getInt("device.navi.threads");
            if (numThreads == 0) {
                numThreads = oc.getInt("device.rerouting.threads");
            }
            router = new CHRouterWrapper<MSEdge, SUMOVehicle>(
                MSEdge::getAllEdges(), true, effortFunc,
                string2time(oc.getString("begin")), string2time(oc.getString("end")), SUMOTime_MAX, hasPermissions, numThreads);
        } else {
            router = new DijkstraRouter<MSEdge, SUMOVehicle>(MSEdge::getAllEdges(), true, effortFunc, nullptr, false, nullptr, true);
        }
        
        RailwayRouter<MSEdge, SUMOVehicle>* railRouter = nullptr;
        if (MSNet::getInstance()->hasBidiEdges()) {
            railRouter = new RailwayRouter<MSEdge, SUMOVehicle>(MSEdge::getAllEdges(), true, effortFunc, nullptr, false, true, false,
                    oc.getFloat("railway.max-train-length"),
                    oc.getFloat("weights.reversal-penalty"));
        }
        const int carWalk = SUMOVehicleParserHelper::parseCarWalkTransfer(oc, false);
        const double taxiWait = STEPS2TIME(string2time(OptionsCont::getOptions().getString("persontrip.taxi.waiting-time")));
        MSTransportableRouter* transRouter = new MSTransportableRouter(MSNet::adaptIntermodalRouter, carWalk, taxiWait, routingAlgorithm, 0);
        myRouterProvider = new MSRouterProvider(router, nullptr, transRouter, railRouter);
        
#ifdef HAVE_FOX
        // Set up router providers for each thread (similar to MSRoutingEngine)
        MFXWorkerThread::Pool& threadPool = MSNet::getInstance()->getEdgeControl().getThreadPool();
        if (threadPool.size() > 0) {
            const std::vector<MFXWorkerThread*>& threads = threadPool.getWorkers();
            if (static_cast<MSEdgeControl::WorkerThread*>(threads.front())->setRouterProvider(myRouterProvider)) {
                for (std::vector<MFXWorkerThread*>::const_iterator t = threads.begin() + 1; t != threads.end(); ++t) {
                    static_cast<MSEdgeControl::WorkerThread*>(*t)->setRouterProvider(myRouterProvider->clone());
                }
            }
        }
#endif
    }
    
    // Get source and destination edges with safety checks (using cached route)
    const MSEdge* source = nullptr;
    const MSEdge* dest = nullptr;
    try {
        if (currentRoute != nullptr && currentRoute->size() > 0) {
            source = currentRoute->getEdges().front();
            dest = currentRoute->getEdges().back();
        }
    } catch (const std::exception& e) {
        WRITE_WARNING("findAndSelectRoute: Failed to get source/dest for vehicle '" + vehicle.getID() + "': " + e.what());
        return nullptr;
    } catch (...) {
        WRITE_WARNING("findAndSelectRoute: Failed to get source/dest for vehicle '" + vehicle.getID() + "'");
        return nullptr;
    }
    
    if (source == nullptr || dest == nullptr || source == dest) {
        return nullptr;
    }
    
    // Check route cache (for TAZ connectors and frequently used OD pairs)
#ifdef HAVE_FOX
    {
        FXMutexLock lock(myRouteCacheMutex);
        auto cacheKey = std::make_pair(source, dest);
        auto cacheIt = myCachedRoutes.find(cacheKey);
        if (cacheIt != myCachedRoutes.end() && cacheIt->second != nullptr) {
            try {
                if (cacheIt->second->size() > 0) {
                    // Calculate cost for cached route
                    const ConstMSEdgeVector& edges = cacheIt->second->getEdges();
                    for (size_t i = 0; i < edges.size(); ++i) {
                        if (edges[i] != nullptr) {
                            newCost += getEffort(edges[i], &vehicle, currentTime);
                        }
                    }
                    // Populate allAlternatives if requested (single cached route with probability 1.0)
                    if (allAlternatives != nullptr) {
                        allAlternatives->emplace_back(cacheIt->second, newCost, 1.0, true);
                    }
                    return cacheIt->second;
                }
            } catch (...) {
                // Cache access failed, proceed to compute route
            }
        }
    }
#endif
    
    // Find k-shortest paths
    std::vector<RouteAlternative> alternatives = findKShortestPaths(source, dest, vehicle, currentTime, myMaxAlternatives);
    
    if (alternatives.empty()) {
        return nullptr;
    }
    
    // Select route using logit model (this calculates probabilities)
    ConstMSRoutePtr selectedRoute = selectRouteByLogit(alternatives, vehicle, currentTime);
    
    if (selectedRoute == nullptr) {
        return nullptr;
    }
    
    // Find the cost of the selected route and populate alternatives output
    newCost = 0.0;
    for (const auto& alt : alternatives) {
        bool isSelected = (alt.route == selectedRoute);
        if (isSelected) {
            newCost = alt.cost;
        }
        // Populate allAlternatives if requested
        if (allAlternatives != nullptr) {
            allAlternatives->emplace_back(alt.route, alt.cost, alt.probability, isSelected);
        }
    }
    
    // Cache route (limit cache size to avoid memory issues)
#ifdef HAVE_FOX
    {
        FXMutexLock lock(myRouteCacheMutex);
        if (myCachedRoutes.size() < 10000) { // Limit cache size
            auto cacheKey = std::make_pair(source, dest);
            if (myCachedRoutes.find(cacheKey) == myCachedRoutes.end()) {
                myCachedRoutes[cacheKey] = selectedRoute;
            }
        }
    }
#endif
    
    return selectedRoute;
    } catch (const std::exception& e) {
        WRITE_WARNING("findAndSelectRoute error: " + std::string(e.what()));
        return nullptr;
    } catch (...) {
        WRITE_WARNING("findAndSelectRoute unknown error");
        return nullptr;
    }
}

MSNaviEngine::RerouteResult
MSNaviEngine::reroute(SUMOVehicle& vehicle, const SUMOTime currentTime, const std::string& info,
                     const bool onInit) {
    RerouteResult result;
    
    try {
        // Safety check: verify vehicle is valid
        try {
            if (vehicle.getRoute().size() == 0) {
                return result;
            }
        } catch (...) {
            return result;
        }
        
        initEdgeWeights(vehicle.getVClass());
        
        // Note: Parallel routing via thread pool is disabled for now due to thread-safety concerns
        // with shared static state (myHistoricalTravelTimes, myRouterProvider).
        // The k-shortest path algorithm with penalty method is not thread-safe.
        // TODO: Implement proper thread-safe routing with thread-local routers
        
        // Sequential routing (fallback)
        // Calculate cost of current remaining route
        double oldCost = 0.0;
        if (!onInit && vehicle.hasDeparted()) {
            try {
                // Cache route pointer to avoid race conditions
                ConstMSRoutePtr route = vehicle.getRoutePtr();
                if (route != nullptr && route->size() > 0) {
                    // Get remaining edges from current position to end
                    const ConstMSEdgeVector& edges = route->getEdges();
                    int routePos = vehicle.getRoutePosition();
                    if (routePos >= 0 && routePos < (int)edges.size()) {
                        for (int i = routePos; i < (int)edges.size(); ++i) {
                            const MSEdge* e = edges[i];
                            if (e != nullptr) {
                                oldCost += getEffort(e, &vehicle, currentTime);
                            }
                        }
                    }
                }
            } catch (const std::exception& e) {
                WRITE_WARNING("reroute: Failed to compute old cost for vehicle '" + vehicle.getID() + "': " + e.what());
                oldCost = 0.0;
            } catch (...) {
                // If we can't compute old cost, just use 0
                oldCost = 0.0;
            }
        }
        
        // Find and select new route, collecting all alternatives
        double newCost = 0.0;
        std::vector<AlternativeInfo> alternatives;
        ConstMSRoutePtr selectedRoute = findAndSelectRoute(vehicle, currentTime, newCost, onInit, &alternatives);
        
        // Store old cost and alternatives in result
        result.oldCost = oldCost;
        result.alternatives = alternatives;
        
        if (selectedRoute == nullptr || selectedRoute->size() == 0) {
            return result;
        }
        
        // Check threshold before rerouting
        double thresholdFactor = vehicle.getFloatParam("device.navi.threshold.factor", true, 1.0);
        double thresholdTime = STEPS2TIME(vehicle.getTimeParam("device.navi.threshold.constant", true, 0));
        
        bool shouldReroute = onInit;
        if (!onInit && oldCost > 0) {
            if (newCost == 0) {
                shouldReroute = true;
            } else {
                shouldReroute = (oldCost / newCost > thresholdFactor) && (oldCost - newCost > thresholdTime);
            }
        }
        
        if (shouldReroute) {
            try {
                // Additional safety check for MESO: ensure vehicle is in valid state
                if (MSGlobals::gUseMesoSim) {
                    // In MESO, check if vehicle is on a valid segment
                    if (!vehicle.isOnRoad() && !onInit) {
                        // Vehicle is not on road (arrived or not yet departed), skip reroute
                        return result;
                    }
                }
                
                std::string errorMsg;
                bool replaceSuccess = vehicle.replaceRoute(selectedRoute, info, onInit, 0, true, true, &errorMsg);
                if (replaceSuccess) {
                    result.success = true;
                    result.route = selectedRoute;
                    result.cost = newCost;
                } else {
                    if (!errorMsg.empty()) {
                        WRITE_WARNING("Navi replaceRoute failed for vehicle '" + vehicle.getID() + "': " + errorMsg);
                    }
                }
            } catch (const std::exception& e) {
                WRITE_WARNING("Navi replaceRoute exception for vehicle '" + vehicle.getID() + "': " + e.what());
            } catch (...) {
                WRITE_WARNING("Navi replaceRoute unknown error for vehicle '" + vehicle.getID() + "'");
            }
        }
    } catch (const ProcessError& e) {
        WRITE_WARNING("Navi routing error for vehicle '" + vehicle.getID() + "': " + e.what());
    } catch (const std::exception& e) {
        WRITE_WARNING("Navi routing exception for vehicle '" + vehicle.getID() + "': " + e.what());
    } catch (...) {
        WRITE_WARNING("Navi routing unknown error for vehicle '" + vehicle.getID() + "'");
    }
    
    return result;
}

std::vector<MSNaviEngine::RouteAlternative>
MSNaviEngine::findKShortestPaths(const MSEdge* from, const MSEdge* to,
                                 SUMOVehicle& vehicle, const SUMOTime currentTime, int k) {
    std::vector<RouteAlternative> alternatives;
    
    // Safety checks
    if (from == nullptr || to == nullptr || k < 1) {
        return alternatives;
    }
    
    // Skip if source equals destination
    if (from == to) {
        return alternatives;
    }
    
#ifdef HAVE_FOX
    // Lock for thread-safe router access
    // The k-shortest path algorithm uses shared routers and penalties, so it must be serialized
    FXMutexLock lock(myDataMutex);
#endif
    
    alternatives.reserve(k); // Pre-allocate space
    
    // For k=1, just use the main router (faster, no penalty overhead)
    if (k == 1) {
        if (myRouterProvider != nullptr) {
            ConstMSEdgeVector edges;
            try {
                auto& router = myRouterProvider->getVehicleRouter(vehicle.getVClass());
                bool found = router.compute(from, to, &vehicle, currentTime, edges);
                if (found && !edges.empty()) {
                    double cost = 0.0;
                    for (const MSEdge* e : edges) {
                        cost += getEffort(e, &vehicle, currentTime);
                    }
                    StopParVector stops;
                    MSRoute* route = new MSRoute("navi_" + vehicle.getID() + "_0", edges, false, nullptr, stops);
                    route->setCosts(cost);  // Set cost on route object to avoid default -1
                    alternatives.push_back(RouteAlternative(ConstMSRoutePtr(route), cost));
                }
            } catch (...) {
                // Fall through to k-shortest path finding
            }
        }
        if (!alternatives.empty()) {
            return alternatives;
        }
    }
    
    // Map to store penalties for edges (for k-shortest path finding)
    std::map<const MSEdge*, double> penalties;
    
    // Set thread-local penalties pointer
    g_penalties = &penalties;
    
    try {
        // Create or reuse persistent router for k-shortest paths
        // This avoids creating/destroying routers which would print statistics
        // Use A* if configured, otherwise Dijkstra
        if (myKShortestRouter == nullptr) {
            OptionsCont& oc = OptionsCont::getOptions();
            const std::string routingAlgorithm = oc.getString("routing-algorithm");
            
            if (routingAlgorithm == "astar") {
                // A* is faster for point-to-point queries on road networks
                typedef AStarRouter<MSEdge, SUMOVehicle, MSMapMatcher> AStar;
                myKShortestRouter = new AStar(MSEdge::getAllEdges(), true, penalizedEffortFunc, nullptr, true);
            } else {
                // Default to Dijkstra (works for any network)
                myKShortestRouter = new DijkstraRouter<MSEdge, SUMOVehicle>(
                    MSEdge::getAllEdges(), true, penalizedEffortFunc, nullptr, false, nullptr, true);
            }
        }
        
        // Use the router through the base class interface
        SUMOAbstractRouter<MSEdge, SUMOVehicle>* router = myKShortestRouter;
        
        int routeIndex = 0;
        const int maxIterations = k + 2; // Reduced iterations - stop after k+2 attempts
        int consecutiveDuplicates = 0;
        
        for (int i = 0; i < maxIterations && (int)alternatives.size() < k; ++i) {
            ConstMSEdgeVector edges;
            bool found = false;
            try {
                found = router->compute(from, to, &vehicle, currentTime, edges);
            } catch (...) {
                // Router compute failed, skip this iteration
                break;
            }
            
            if (!found || edges.empty()) {
                break;
            }
        
            // Fast duplicate check using hash-like comparison (size + first + last edge)
            bool isDuplicate = false;
            const size_t edgeCount = edges.size();
            const MSEdge* firstEdge = edges.front();
            const MSEdge* lastEdge = edges.back();
            
            for (const auto& alt : alternatives) {
                // Safety check: ensure route is valid
                if (alt.route == nullptr) {
                    continue;
                }
                try {
                    const ConstMSEdgeVector& altEdges = alt.route->getEdges();
                    if (altEdges.empty()) {
                        continue;
                    }
                    // Quick rejection: different size or different endpoints
                    if (altEdges.size() != edgeCount || altEdges.front() != firstEdge || altEdges.back() != lastEdge) {
                        continue;
                    }
                    // Full comparison only if quick check passes
                    if (altEdges == edges) {
                        isDuplicate = true;
                        break;
                    }
                } catch (...) {
                    continue;
                }
            }
        
        if (isDuplicate) {
            consecutiveDuplicates++;
            // If too many consecutive duplicates, break early
            if (consecutiveDuplicates > 2) {
                break;
            }
            // If duplicate, add larger penalty to divergence edges
            // Only penalize first few edges to find different paths faster
            const size_t penalizeCount = MIN2(edges.size(), (size_t)5);
            for (size_t j = 0; j < penalizeCount; ++j) {
                if (edges[j] != nullptr) {
                    penalties[edges[j]] += getEffort(edges[j], &vehicle, currentTime) * 0.3;
                }
            }
            continue;
        }
        
            consecutiveDuplicates = 0; // Reset counter on finding unique route
            
            // Calculate route cost using actual effort (without penalties)
            double cost = 0.0;
            for (const MSEdge* e : edges) {
                if (e != nullptr) {
                    cost += getEffort(e, &vehicle, currentTime);
                }
            }
        
            // Create route
            try {
                StopParVector stops;
                MSRoute* route = new MSRoute("navi_" + vehicle.getID() + "_" + toString(routeIndex), edges, false, nullptr, stops);
                route->setCosts(cost);  // Set cost on route object to avoid default -1
                ConstMSRoutePtr routePtr(route);
                alternatives.push_back(RouteAlternative(routePtr, cost));
                routeIndex++;
            } catch (...) {
                // Route creation failed, continue to next iteration
                continue;
            }
        
            // Add penalty to first few edges to find different paths faster
            // Penalizing all edges is expensive and not necessary
            if (edges.size() > 1) {
                const size_t penalizeCount = MIN2(edges.size() - 1, (size_t)10);
                const double penaltyFactor = 0.1;
                for (size_t j = 0; j < penalizeCount; ++j) {
                    if (edges[j] != nullptr) {
                        penalties[edges[j]] += cost * penaltyFactor;
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        WRITE_WARNING("findKShortestPaths error: " + std::string(e.what()));
    } catch (...) {
        WRITE_WARNING("findKShortestPaths unknown error");
    }
    
    // Clear thread-local penalties pointer
    g_penalties = nullptr;
    
    return alternatives;
}

ConstMSRoutePtr
MSNaviEngine::selectRouteByLogit(std::vector<RouteAlternative>& alternatives,
                                  SUMOVehicle& vehicle, const SUMOTime currentTime) {
    if (alternatives.empty()) {
        return nullptr;
    }
    
    try {
        if (alternatives.size() == 1) {
            return alternatives[0].route;
        }
        
        // Calculate logit probabilities
        calculateLogitProbabilities(alternatives, myLogitTheta);
        
        // Use vehicle's RNG for probabilistic selection
        double random = RandHelper::rand(vehicle.getRNG());
        double cumulativeProb = 0.0;
        
        for (size_t i = 0; i < alternatives.size(); ++i) {
            cumulativeProb += alternatives[i].probability;
            if (random <= cumulativeProb) {
                return alternatives[i].route;
            }
        }
        
        // Fallback to first route
        return alternatives[0].route;
    } catch (...) {
        // Return first route on any error
        if (!alternatives.empty()) {
            return alternatives[0].route;
        }
        return nullptr;
    }
}

void
MSNaviEngine::calculateLogitProbabilities(std::vector<RouteAlternative>& alternatives, double theta) {
    if (alternatives.empty()) {
        return;
    }
    
    try {
        // Find minimum cost
        double minCost = std::numeric_limits<double>::max();
        for (size_t i = 0; i < alternatives.size(); ++i) {
            if (alternatives[i].cost < minCost) {
                minCost = alternatives[i].cost;
            }
        }
        
        // Calculate exponential utilities and sum in one pass
        double sumExpUtilities = 0.0;
        for (size_t i = 0; i < alternatives.size(); ++i) {
            double utility = -theta * (alternatives[i].cost - minCost); // Negative because lower cost is better
            // Clamp utility to prevent overflow in exp()
            utility = MAX2(-700.0, MIN2(700.0, utility));
            double expUtil = exp(utility);
            alternatives[i].probability = expUtil;  // Temporarily store exp utility
            sumExpUtilities += expUtil;
        }
        
        // Normalize to get probabilities
        if (sumExpUtilities > 0) {
            for (size_t i = 0; i < alternatives.size(); ++i) {
                alternatives[i].probability /= sumExpUtilities;
            }
        } else {
            // Fallback: equal probabilities
            double equalProb = 1.0 / alternatives.size();
            for (size_t i = 0; i < alternatives.size(); ++i) {
                alternatives[i].probability = equalProb;
            }
        }
    } catch (...) {
        // Fallback: equal probabilities
        double equalProb = 1.0 / alternatives.size();
        for (size_t i = 0; i < alternatives.size(); ++i) {
            alternatives[i].probability = equalProb;
        }
    }
}

void
MSNaviEngine::addEdgeTravelTime(const MSEdge& edge, const SUMOTime travelTime) {
#ifdef HAVE_FOX
    FXMutexLock lock(myDataMutex);
#endif
    auto it = myCurrentIntervalData.find(&edge);
    if (it != myCurrentIntervalData.end()) {
        it->second.totalTravelTime += STEPS2TIME(travelTime);
        it->second.vehicleCount++;
    }
}

void
MSNaviEngine::saveState(OutputDevice& out) {
    out.openTag("naviEngine");
    out.writeAttr("historyIntervals", myHistoryIntervals);
    out.writeAttr("currentIntervalIndex", myCurrentIntervalIndex);
    out.writeAttr("lastUpdate", time2string(myLastUpdate));
    // Save historical travel times
    for (const auto& pair : myHistoricalTravelTimes) {
        out.openTag("edgeHistory");
        out.writeAttr("id", pair.first->getID());
        const std::deque<IntervalData>& history = pair.second;
        for (size_t i = 0; i < history.size(); ++i) {
            out.openTag("interval");
            out.writeAttr("index", (int)i);
            out.writeAttr("avgTravelTime", history[i].getAverage());
            out.closeTag();
        }
        out.closeTag();
    }
    out.closeTag();
}

void
MSNaviEngine::loadState(const SUMOSAXAttributes& attrs) {
    // Implementation for loading state
    // This would parse the XML attributes and restore the historical travel times
}

#ifdef HAVE_FOX
void
MSNaviEngine::waitForAll() {
#ifndef THREAD_POOL
    MFXWorkerThread::Pool& threadPool = MSNet::getInstance()->getEdgeControl().getThreadPool();
    if (threadPool.size() > 0) {
        threadPool.waitAll();
    }
#endif
}

// ---------------------------------------------------------------------------
// MSNaviEngine::NaviRoutingTask-methods
// ---------------------------------------------------------------------------
void
MSNaviEngine::NaviRoutingTask::run(MFXWorkerThread* context) {
    try {
        // Safety check: verify vehicle and route are valid
        try {
            if (myVehicle.getRoute().size() == 0) {
                return;
            }
        } catch (...) {
            return;
        }
        
        // Calculate cost of current remaining route
        double oldCost = 0.0;
        if (!myOnInit && myVehicle.hasDeparted()) {
            try {
                // Safety check: ensure current route edge iterator is valid
                auto currentEdgeIt = myVehicle.getCurrentRouteEdge();
                auto routeEnd = myVehicle.getRoute().end();
                auto routeBegin = myVehicle.getRoute().begin();
                
                // Validate iterator range before constructing vector
                if (currentEdgeIt >= routeBegin && currentEdgeIt < routeEnd) {
                    ConstMSEdgeVector remainingEdges(currentEdgeIt, routeEnd);
                    for (size_t i = 0; i < remainingEdges.size(); ++i) {
                        const MSEdge* e = remainingEdges[i];
                        if (e != nullptr) {
                            oldCost += MSNaviEngine::getEffort(e, &myVehicle, myTime);
                        }
                    }
                }
            } catch (...) {
                // If we can't compute old cost, just use 0
                oldCost = 0.0;
            }
        }
        
        // Find and select new route
        double newCost = 0.0;
        ConstMSRoutePtr selectedRoute = MSNaviEngine::findAndSelectRoute(myVehicle, myTime, newCost, myOnInit);
        
        if (selectedRoute == nullptr) {
            return;
        }
        
        try {
            if (selectedRoute->size() == 0) {
                return;
            }
        } catch (...) {
            return;
        }
        
        // Check threshold before rerouting (need to get threshold from device)
        // For now, we'll skip threshold check in parallel mode or get it from vehicle parameters
        double thresholdFactor = myVehicle.getFloatParam("device.navi.threshold.factor", true, 1.0);
        double thresholdTime = STEPS2TIME(myVehicle.getTimeParam("device.navi.threshold.constant", true, 0));
        
        bool shouldReroute = myOnInit;
        if (!myOnInit && oldCost > 0) {
            if (newCost == 0) {
                shouldReroute = true;
            } else {
                shouldReroute = (oldCost / newCost > thresholdFactor) && (oldCost - newCost > thresholdTime);
            }
        }
        
        if (shouldReroute) {
            myVehicle.replaceRoute(selectedRoute, myInfo, myOnInit);
        }
        
        // Cache route (limit cache size to avoid memory issues)
        try {
            if (myVehicle.getRoute().size() > 0) {
                const MSEdge* source = *myVehicle.getRoute().begin();
                const MSEdge* dest = myVehicle.getRoute().getLastEdge();
                if (source != nullptr && dest != nullptr) {
                    FXMutexLock lock(myRouteCacheMutex);
                    if (myCachedRoutes.size() < 10000) { // Limit cache size
                        auto cacheKey = std::make_pair(source, dest);
                        if (myCachedRoutes.find(cacheKey) == myCachedRoutes.end()) {
                            myCachedRoutes[cacheKey] = selectedRoute;
                        }
                    }
                }
            }
        } catch (...) {
            // Cache error is not critical, ignore
        }
    } catch (const std::exception& e) {
        WRITE_WARNING("NaviRoutingTask error for vehicle '" + myVehicle.getID() + "': " + e.what());
    } catch (...) {
        WRITE_WARNING("NaviRoutingTask unknown error for vehicle '" + myVehicle.getID() + "'");
    }
}
#endif

void
MSNaviEngine::cleanup() {
    // Note: StaticCommand is managed by MSEventControl, so we just set pointer to nullptr
    // The event system will handle cleanup when the command returns 0 or negative
    myTravelTimeUpdateCommand = nullptr;
    if (myRouterProvider != nullptr) {
        delete myRouterProvider;
        myRouterProvider = nullptr;
    }
    if (myKShortestRouter != nullptr) {
        delete myKShortestRouter;
        myKShortestRouter = nullptr;
    }
    myHistoricalTravelTimes.clear();
    myCurrentIntervalData.clear();
#ifdef HAVE_FOX
    myCachedRoutes.clear();
#endif
    myInitialized = false;
}

