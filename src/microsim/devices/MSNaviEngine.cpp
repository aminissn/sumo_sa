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
    // Initialize historical travel times for all edges
    const MSEdgeVector& edges = MSNet::getInstance()->getEdgeControl().getEdges();
    for (const MSEdge* edge : edges) {
        if (myHistoricalTravelTimes.find(edge) == myHistoricalTravelTimes.end()) {
            // Initialize with empty deque
            myHistoricalTravelTimes[edge] = std::deque<IntervalData>();
            // Fill with initial data based on edge's free-flow travel time
            const double freeFlowTT = edge->getLength() / edge->getSpeedLimit();
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
    // Save current interval data to history
    for (auto& pair : myCurrentIntervalData) {
        const MSEdge* edge = pair.first;
        IntervalData& data = pair.second;
        
        // Calculate average for this interval
        double avgTT = data.getAverage();
        if (avgTT == 0.0) {
            // No data, use free-flow travel time
            avgTT = edge->getLength() / edge->getSpeedLimit();
        }
        
        // Add to history
        std::deque<IntervalData>& history = myHistoricalTravelTimes[edge];
        if ((int)history.size() >= myHistoryIntervals) {
            history.pop_front();
        }
        IntervalData intervalData;
        intervalData.totalTravelTime = avgTT;
        intervalData.vehicleCount = 1;
        history.push_back(intervalData);
        
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
    
    return UPDATE_INTERVAL;
}

double
MSNaviEngine::getEffort(const MSEdge* const e, const SUMOVehicle* const v, double t) {
    // Return average travel time over historical intervals
    return getAverageTravelTime(e, myHistoryIntervals);
}

double
MSNaviEngine::getAverageTravelTime(const MSEdge* edge, int intervals) {
    auto it = myHistoricalTravelTimes.find(edge);
    if (it == myHistoricalTravelTimes.end()) {
        // No history, return free-flow travel time
        return edge->getLength() / edge->getSpeedLimit();
    }
    
    const std::deque<IntervalData>& history = it->second;
    if (history.empty()) {
        return edge->getLength() / edge->getSpeedLimit();
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
                                  double& newCost, const bool onInit) {
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
    
    // Get source and destination edges
    const MSEdge* source = *vehicle.getRoute().begin();
    const MSEdge* dest = vehicle.getRoute().getLastEdge();
    
    if (source == nullptr || dest == nullptr) {
        return nullptr;
    }
    
    // Check route cache first (for TAZ connectors)
    if (source->isTazConnector() && dest->isTazConnector()) {
#ifdef HAVE_FOX
        FXMutexLock lock(myRouteCacheMutex);
#endif
        auto cacheKey = std::make_pair(source, dest);
        auto cacheIt = myCachedRoutes.find(cacheKey);
        if (cacheIt != myCachedRoutes.end()) {
            // Calculate cost for cached route
            for (const MSEdge* e : cacheIt->second->getEdges()) {
                newCost += getEffort(e, &vehicle, currentTime);
            }
            return cacheIt->second;
        }
    }
    
    // Find k-shortest paths
    std::vector<RouteAlternative> alternatives = findKShortestPaths(source, dest, vehicle, currentTime, myMaxAlternatives);
    
    if (alternatives.empty()) {
        return nullptr;
    }
    
    // Select route using logit model
    ConstMSRoutePtr selectedRoute = selectRouteByLogit(alternatives, vehicle, currentTime);
    
    if (selectedRoute == nullptr) {
        return nullptr;
    }
    
    // Find the cost of the selected route
    newCost = 0.0;
    for (const auto& alt : alternatives) {
        if (alt.route == selectedRoute) {
            newCost = alt.cost;
            break;
        }
    }
    
    // Cache route for TAZ connectors
    if (source->isTazConnector() && dest->isTazConnector()) {
#ifdef HAVE_FOX
        FXMutexLock lock(myRouteCacheMutex);
#endif
        auto cacheKey = std::make_pair(source, dest);
        if (myCachedRoutes.find(cacheKey) == myCachedRoutes.end()) {
            myCachedRoutes[cacheKey] = selectedRoute;
        }
    }
    
    return selectedRoute;
}

void
MSNaviEngine::reroute(SUMOVehicle& vehicle, const SUMOTime currentTime, const std::string& info,
                     const bool onInit) {
    initEdgeWeights(vehicle.getVClass());
    
#ifdef HAVE_FOX
    // Use parallel routing if thread pool is available
    MFXWorkerThread::Pool& threadPool = MSNet::getInstance()->getEdgeControl().getThreadPool();
    if (threadPool.size() > 0) {
        threadPool.add(new NaviRoutingTask(vehicle, currentTime, info, onInit));
        return;
    }
#endif
    
    // Sequential routing (fallback)
    // Calculate cost of current remaining route
    double oldCost = 0.0;
    if (!onInit && vehicle.hasDeparted()) {
        ConstMSEdgeVector remainingEdges(vehicle.getCurrentRouteEdge(), vehicle.getRoute().end());
        if (!remainingEdges.empty()) {
            for (const MSEdge* e : remainingEdges) {
                oldCost += getEffort(e, &vehicle, currentTime);
            }
        }
    }
    
    // Find and select new route
    double newCost = 0.0;
    ConstMSRoutePtr selectedRoute = findAndSelectRoute(vehicle, currentTime, newCost, onInit);
    
    if (selectedRoute == nullptr) {
        return;
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
        vehicle.replaceRoute(selectedRoute, info, onInit);
    }
}

std::vector<MSNaviEngine::RouteAlternative>
MSNaviEngine::findKShortestPaths(const MSEdge* from, const MSEdge* to,
                                 SUMOVehicle& vehicle, const SUMOTime currentTime, int k) {
    std::vector<RouteAlternative> alternatives;
    alternatives.reserve(k); // Pre-allocate space
    
    // Map to store penalties for edges (for k-shortest path finding)
    std::map<const MSEdge*, double> penalties;
    
    // Set thread-local penalties pointer
    g_penalties = &penalties;
    
    // Create or reuse persistent router for k-shortest paths
    // This avoids creating/destroying routers which would print statistics
    if (myKShortestRouter == nullptr) {
        myKShortestRouter = new DijkstraRouter<MSEdge, SUMOVehicle>(
            MSEdge::getAllEdges(), true, penalizedEffortFunc, nullptr, false, nullptr, true);
    }
    
    // Cast to the specific router type we need
    DijkstraRouter<MSEdge, SUMOVehicle>* router = 
        static_cast<DijkstraRouter<MSEdge, SUMOVehicle>*>(myKShortestRouter);
    
    int routeIndex = 0;
    const int maxIterations = k * 2; // Reduced from k*3 for better performance
    int consecutiveDuplicates = 0;
    
    for (int i = 0; i < maxIterations && (int)alternatives.size() < k; ++i) {
        ConstMSEdgeVector edges;
        bool found = router->compute(from, to, &vehicle, currentTime, edges);
        
        if (!found || edges.empty()) {
            break;
        }
        
        // Fast duplicate check: compare routes directly (vector comparison is optimized)
        bool isDuplicate = false;
        for (const auto& alt : alternatives) {
            if (alt.route->getEdges() == edges) {
                isDuplicate = true;
                break;
            }
        }
        
        if (isDuplicate) {
            consecutiveDuplicates++;
            // If too many consecutive duplicates, break early
            if (consecutiveDuplicates > 3) {
                break;
            }
            // If duplicate, add larger penalty to try to find different route
            for (const MSEdge* e : edges) {
                penalties[e] += getEffort(e, &vehicle, currentTime) * 0.5;
            }
            continue;
        }
        
        consecutiveDuplicates = 0; // Reset counter on finding unique route
        
        // Calculate route cost using actual effort (without penalties)
        double cost = 0.0;
        for (const MSEdge* e : edges) {
            cost += getEffort(e, &vehicle, currentTime);
        }
        
        // Create route
        StopParVector stops;
        MSRoute* route = new MSRoute("navi_" + vehicle.getID() + "_" + toString(routeIndex), edges, false, nullptr, stops);
        ConstMSRoutePtr routePtr(route);
        alternatives.push_back(RouteAlternative(routePtr, cost));
        routeIndex++;
        
        // Add penalty to edges in this route (except the last one) to find different paths
        // Use a smaller penalty to avoid over-penalizing
        const double penaltyFactor = 0.05;
        for (size_t j = 0; j < edges.size() - 1; ++j) {
            penalties[edges[j]] += cost * penaltyFactor;
        }
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
    
    if (alternatives.size() == 1) {
        return alternatives[0].route;
    }
    
    // Calculate logit probabilities
    calculateLogitProbabilities(alternatives, myLogitTheta);
    
    // Use vehicle's RNG for probabilistic selection
    double random = RandHelper::rand(vehicle.getRNG());
    double cumulativeProb = 0.0;
    
    for (auto& alt : alternatives) {
        cumulativeProb += alt.probability;
        if (random <= cumulativeProb) {
            return alt.route;
        }
    }
    
    // Fallback to first route
    return alternatives[0].route;
}

void
MSNaviEngine::calculateLogitProbabilities(std::vector<RouteAlternative>& alternatives, double theta) {
    if (alternatives.empty()) {
        return;
    }
    
    // Find minimum cost
    double minCost = std::numeric_limits<double>::max();
    for (const auto& alt : alternatives) {
        if (alt.cost < minCost) {
            minCost = alt.cost;
        }
    }
    
    // Calculate exponential utilities
    std::vector<double> expUtilities;
    double sumExpUtilities = 0.0;
    
    for (const auto& alt : alternatives) {
        double utility = -theta * (alt.cost - minCost); // Negative because lower cost is better
        double expUtil = exp(utility);
        expUtilities.push_back(expUtil);
        sumExpUtilities += expUtil;
    }
    
    // Calculate probabilities
    for (size_t i = 0; i < alternatives.size(); ++i) {
        alternatives[i].probability = expUtilities[i] / sumExpUtilities;
    }
}

void
MSNaviEngine::addEdgeTravelTime(const MSEdge& edge, const SUMOTime travelTime) {
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
    // Calculate cost of current remaining route
    double oldCost = 0.0;
    if (!myOnInit && myVehicle.hasDeparted()) {
        ConstMSEdgeVector remainingEdges(myVehicle.getCurrentRouteEdge(), myVehicle.getRoute().end());
        if (!remainingEdges.empty()) {
            for (const MSEdge* e : remainingEdges) {
                oldCost += MSNaviEngine::getEffort(e, &myVehicle, myTime);
            }
        }
    }
    
    // Find and select new route
    double newCost = 0.0;
    ConstMSRoutePtr selectedRoute = MSNaviEngine::findAndSelectRoute(myVehicle, myTime, newCost, myOnInit);
    
    if (selectedRoute == nullptr) {
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
    
    // Cache route for TAZ connectors
    const MSEdge* source = *myVehicle.getRoute().begin();
    const MSEdge* dest = myVehicle.getRoute().getLastEdge();
    if (source->isTazConnector() && dest->isTazConnector()) {
        FXMutexLock lock(myRouteCacheMutex);
        auto cacheKey = std::make_pair(source, dest);
        if (myCachedRoutes.find(cacheKey) == myCachedRoutes.end()) {
            myCachedRoutes[cacheKey] = selectedRoute;
        }
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

