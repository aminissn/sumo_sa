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
/// @file    MSODRouteExport.h
/// @author  aminissn
/// @date    2026-09-14
///
// Aggregated route flows for each origin-destination (TAZ) pair
/****************************************************************************/
#pragma once
#include <config.h>

#include <map>
#include <string>
#include <vector>
#include <utils/common/SUMOTime.h>
#include <microsim/MSRoute.h>


// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class SUMOVehicle;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSODRouteExport
 * @brief Counts how many vehicles used each distinct route between every
 *        origin-destination pair and writes the result as
 *        interval > tazRelation|edgeRelation > route (option --od-route-output).
 *
 * Counting happens once per vehicle on arrival (in MSVehicleControl::removePending)
 * so the per-step simulation cost is unaffected. All routes with the same edge
 * sequence are merged into a single entry regardless of their route id.
 *
 * The origin and destination are the TAZ which contain the first and the last
 * edge of the route according to the loaded TAZ definitions (source edges for
 * the origin, sink edges for the destination). The vehicle attributes fromTaz /
 * toTaz are deliberately ignored. Edges which are not part of any TAZ are
 * reported with their own id. If no TAZ are loaded or option
 * --od-route-output.edges is set, the departure and arrival edges are used directly.
 */
class MSODRouteExport {
public:
    /// @brief Initialises the output device (if the option is set) and reads sub-options
    static void init();

    /// @brief Returns whether the output is active
    static bool active() {
        return myActive;
    }

    /// @brief Registers an arrived (or, at simulation end, still running) vehicle
    static void addVehicle(const SUMOVehicle& veh, bool arrived = true);

    /// @brief Called after every simulation step, writes the interval if a period is set and complete
    static void write(SUMOTime step);

    /// @brief Called at simulation end, optionally adds unfinished vehicles and writes the remaining interval
    static void finish(SUMOTime step);

    /// @brief Resets all static state (for repeated simulations within one process)
    static void cleanup();

private:
    /// @brief per route counts
    struct RouteCount {
        ConstMSEdgeVector edges;
        double length = 0;
        int count = 0;
        /// @brief number of vehicles which completed the route (used for the mean travel time)
        int arrived = 0;
        /// @brief sum of travel times of the arrived vehicles
        double travelTime = 0;
    };

    /// @brief per origin-destination counts; routes are keyed by the numerical ids of their edges
    struct ODCount {
        int count = 0;
        std::map<std::vector<int>, RouteCount> routes;
    };

    /// @brief writes the collected counts for the given interval and clears them
    static void writeInterval(SUMOTime begin, SUMOTime end);

    /// @brief returns the origin (departure edge) or destination (arrival edge) id for the given edge
    static const std::string& getZone(const MSEdge* edge, bool origin);

private:
    /// @brief whether the output is active
    static bool myActive;

    /// @brief whether vehicles which have not arrived at simulation end shall be counted
    static bool myWriteUnfinished;

    /// @brief the aggregation period (<= 0 for the whole simulation)
    static SUMOTime myPeriod;

    /// @brief the begin of the current interval
    static SUMOTime myIntervalBegin;

    /// @brief whether at least one interval has been written
    static bool myWroteInterval;

    /// @brief whether the edges shall be mapped to TAZ (-1: not yet determined, 0: use edges, 1: use TAZ)
    static int myUseTaz;

    /// @brief cached mapping of departure edges to origin ids
    static std::map<const MSEdge*, std::string> myOrigins;

    /// @brief cached mapping of arrival edges to destination ids
    static std::map<const MSEdge*, std::string> myDestinations;

    /// @brief the collected counts of the current interval, keyed by (fromTaz, toTaz)
    static std::map<std::pair<std::string, std::string>, ODCount> myCounts;
};
