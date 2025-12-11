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
/// @file    MSNaviEngine.h
/// @author  Sasan Amini    
/// @date    2025-12-10
///
// Engine for device.navi that manages historical travel times and route finding
/****************************************************************************/
#pragma once
#include <config.h>

#include <vector>
#include <map>
#include <deque>
#include <utils/common/SUMOTime.h>
#include <utils/common/WrappingCommand.h>
#include <microsim/MSRouterDefs.h>
#ifdef HAVE_FOX
#include <utils/foxtools/MFXWorkerThread.h>
#endif

// ===========================================================================
// class declarations
// ===========================================================================
class MSEdge;
class SUMOVehicle;
class OutputDevice;
class SUMOSAXAttributes;
class MSDevice_Navi;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSNaviEngine
 * @brief Engine for device.navi that manages historical travel times and route finding
 *
 * This engine:
 * - Calculates average travel time every 60 seconds
 * - Stores historical travel times for x intervals (user-configurable)
 * - Provides edge weights based on average travel time over past x intervals
 * - Finds k-shortest paths
 * - Applies logit model for probabilistic route choice
 */
class MSNaviEngine {
public:
    /// @brief Initialize the navi engine
    static void init();

    /// @brief Initialize edge weights if not done before
    static void initEdgeWeights(SUMOVehicleClass svc);

    /// @brief Update travel times every 60 seconds
    static SUMOTime updateTravelTimes(SUMOTime currentTime);

    /// @brief Get edge effort (travel time) based on historical averages
    static double getEffort(const MSEdge* const e, const SUMOVehicle* const v, double t);

    /// @brief Find k-shortest paths and select one using logit model
    /// @return The selected route (nullptr if no route found)
    /// @param[out] newCost The cost of the selected route (if route was found)
    static ConstMSRoutePtr findAndSelectRoute(SUMOVehicle& vehicle, const SUMOTime currentTime, 
                                    double& newCost, const bool onInit = false);

    /// @brief Reroute a vehicle (with parallelization support)
    static void reroute(SUMOVehicle& vehicle, const SUMOTime currentTime, const std::string& info,
                        const bool onInit = false);

    /// @brief Record actual travel time for an edge
    static void addEdgeTravelTime(const MSEdge& edge, const SUMOTime travelTime);

    /// @brief Get the number of historical intervals to look back
    static int getHistoryIntervals() {
        return myHistoryIntervals;
    }

    /// @brief Get logit theta parameter
    static double getLogitTheta() {
        return myLogitTheta;
    }

    /// @brief Get number of alternative routes to consider
    static int getMaxAlternatives() {
        return myMaxAlternatives;
    }

    /// @brief Get last update time
    static SUMOTime getLastUpdate() {
        return myLastUpdate;
    }

    /// @brief Saves the state (i.e. recorded travel times)
    static void saveState(OutputDevice& out);

    /// @brief Loads the state
    static void loadState(const SUMOSAXAttributes& attrs);

    /// @brief Cleanup
    static void cleanup();

#ifdef HAVE_FOX
    /// @brief Wait for all routing tasks to complete
    static void waitForAll();
#endif

private:
#ifdef HAVE_FOX
    /**
     * @class NaviRoutingTask
     * @brief the routing task for parallel execution
     */
    class NaviRoutingTask : public MFXWorkerThread::Task {
    public:
        NaviRoutingTask(SUMOVehicle& v, const SUMOTime time, const std::string& info, const bool onInit)
            : myVehicle(v), myTime(time), myInfo(info), myOnInit(onInit) {}
        void run(MFXWorkerThread* context);
    private:
        SUMOVehicle& myVehicle;
        const SUMOTime myTime;
        const std::string myInfo;
        const bool myOnInit;
    private:
        /// @brief Invalidated assignment operator.
        NaviRoutingTask& operator=(const NaviRoutingTask&) = delete;
    };
#endif

private:
    /// @brief Structure to store travel time data for a 60-second interval
    struct IntervalData {
        double totalTravelTime;
        int vehicleCount;
        
        IntervalData() : totalTravelTime(0.0), vehicleCount(0) {}
        
        double getAverage() const {
            return vehicleCount > 0 ? totalTravelTime / vehicleCount : 0.0;
        }
    };

    /// @brief Structure to store a route alternative with cost
    struct RouteAlternative {
        ConstMSRoutePtr route;
        double cost;
        double probability;
        
        RouteAlternative(ConstMSRoutePtr r, double c) : route(r), cost(c), probability(0.0) {}
    };

    /// @brief Calculate average travel time over historical intervals for an edge
    static double getAverageTravelTime(const MSEdge* edge, int intervals);

    /// @brief Find k-shortest paths using penalty method
    static std::vector<RouteAlternative> findKShortestPaths(
        const MSEdge* from, const MSEdge* to, 
        SUMOVehicle& vehicle, const SUMOTime currentTime, int k);

    /// @brief Apply logit model to select route from alternatives
    static ConstMSRoutePtr selectRouteByLogit(
        std::vector<RouteAlternative>& alternatives, 
        SUMOVehicle& vehicle, const SUMOTime currentTime);

    /// @brief Calculate logit probabilities
    static void calculateLogitProbabilities(
        std::vector<RouteAlternative>& alternatives, double theta);

private:
    /// @brief Command for updating travel times every 60 seconds
    static Command* myTravelTimeUpdateCommand;

    /// @brief Interval for updating travel times (60 seconds)
    static const SUMOTime UPDATE_INTERVAL;

    /// @brief Number of historical intervals to look back (user-configurable)
    static int myHistoryIntervals;

    /// @brief Logit model theta parameter (user-configurable)
    static double myLogitTheta;

    /// @brief Maximum number of alternative routes to consider (user-configurable)
    static int myMaxAlternatives;

    /// @brief Current interval index (ring buffer)
    static int myCurrentIntervalIndex;

    /// @brief Historical travel time data: edge -> deque of interval data
    /// Each deque stores data for the last myHistoryIntervals intervals
    static std::map<const MSEdge*, std::deque<IntervalData> > myHistoricalTravelTimes;

    /// @brief Current interval data being collected
    static std::map<const MSEdge*, IntervalData> myCurrentIntervalData;

    /// @brief Last update time
    static SUMOTime myLastUpdate;

    /// @brief Router provider
    static MSRouterProvider* myRouterProvider;

    /// @brief Persistent router for k-shortest paths (reused to avoid statistics output)
    static SUMOAbstractRouter<MSEdge, SUMOVehicle>* myKShortestRouter;

    /// @brief Whether the engine has been initialized
    static bool myInitialized;

#ifdef HAVE_FOX
    /// @brief Mutex for accessing route cache
    static FXMutex myRouteCacheMutex;
    
    /// @brief Route cache for common OD pairs
    static std::map<std::pair<const MSEdge*, const MSEdge*>, ConstMSRoutePtr> myCachedRoutes;
#endif

private:
    /// @brief Invalidated copy constructor.
    MSNaviEngine(const MSNaviEngine&);

    /// @brief Invalidated assignment operator.
    MSNaviEngine& operator=(const MSNaviEngine&);
};

