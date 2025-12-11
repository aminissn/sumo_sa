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
/// @file    MSDevice_Navi.h
/// @author  Sasan Amini
/// @date    2025-12-10
///
// A device that performs vehicle rerouting using historical travel times and logit model
/****************************************************************************/
#pragma once
#include <config.h>

#include <set>
#include <vector>
#include <map>
#include <utils/common/SUMOTime.h>
#include <utils/common/WrappingCommand.h>
#include <microsim/MSVehicle.h>
#include "MSVehicleDevice.h"

// ===========================================================================
// class declarations
// ===========================================================================
class MSLane;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_Navi
 * @brief A device that performs vehicle rerouting using historical travel times and logit model
 *
 * This device:
 * - Uses average travel time from past x intervals (60-second intervals)
 * - Finds k-shortest paths
 * - Applies logit model for probabilistic route choice
 * - Supports random seed for reproducibility
 * - Supports output like device.rerouting
 */
class MSDevice_Navi : public MSVehicleDevice {
public:
    /** @brief Inserts MSDevice_Navi-options
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);

    /** @brief checks MSDevice_Navi-options
     * @param[filled] oc The options container with the user-defined options
     */
    static bool checkOptions(OptionsCont& oc);

    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether navi-devices shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /// @brief Destructor.
    ~MSDevice_Navi();

    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    /// @{

    /** @brief Computes a new route on vehicle insertion
     *
     * A new route is computed using historical travel times and logit model.
     *
     * If the reroute period is larger than 0, an event is generated and added
     *  to the list of simulation step begin events which executes
     *  "wrappedRerouteCommandExecute".
     *
     * @param[in] veh The entering vehicle.
     * @param[in] reason how the vehicle enters the lane
     * @return Always false
     * @see MSMoveReminder::notifyEnter
     */
    bool notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);

    /// @brief called to do the rerouting we missed while stopping
    void notifyStopEnded();

    /// @}

    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "navi";
    }

    /** @brief Saves the state of the device
     *
     * @param[in] out The OutputDevice to write the information into
     */
    void saveState(OutputDevice& out) const;

    /** @brief Loads the state of the device from the given description
     *
     * @param[in] attrs XML attributes describing the current state
     */
    void loadState(const SUMOSAXAttributes& attrs);

    /// @brief initiate the rerouting
    void reroute(const SUMOTime currentTime, const bool onInit = false);

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    void setParameter(const std::string& key, const std::string& value);

private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     * @param[in] period The period with which a new route shall be searched
     * @param[in] preInsertionPeriod The route search period before insertion
     */
    MSDevice_Navi(SUMOVehicle& holder, const std::string& id, SUMOTime period, SUMOTime preInsertionPeriod);

    /** @brief Performs rerouting before insertion into the network
     *
     * @param[in] currentTime The current simulation time
     * @return The offset to the next call (the rerouting period "myPreInsertionPeriod")
     */
    SUMOTime preInsertionReroute(const SUMOTime currentTime);

    /** @brief Performs rerouting after a period
     *
     * @param[in] currentTime The current simulation time
     * @return The offset to the next call (the rerouting period "myPeriod")
     */
    SUMOTime wrappedRerouteCommandExecute(SUMOTime currentTime);

    /// @brief rebuild reroute command according to period
    void rebuildRerouteCommand(SUMOTime start);

private:
    /// @brief The period with which a vehicle shall be rerouted
    SUMOTime myPeriod;

    /// @brief The period with which a vehicle shall be rerouted before insertion
    SUMOTime myPreInsertionPeriod;

    /// @brief The last time a routing took place
    SUMOTime myLastRouting;

    /// @brief The time for which routing may be skipped because we cannot be inserted
    SUMOTime mySkipRouting;

    /// @brief The (optional) command responsible for rerouting
    WrappingCommand< MSDevice_Navi >* myRerouteCommand;

    /// @brief the previous time that a vehicle entered a lane
    SUMOTime myLastLaneEntryTime;

    /// @brief Whether the equipped vehicle missed a reroute while stopping and should do so after the stop has ended
    bool myRerouteAfterStop;

    /// @brief Only reroute if the new route is faster than the current route by factor
    double myThresholdFactor;

    /// @brief Only reroute if the new route is faster than the current route by seconds
    double myThresholdTime;

    /// @brief whether the change in saving is enough to trigger rerouting
    bool sufficientSaving(double oldCost, double newCost);

    /// @brief Write route to output device
    void writeRoute(const ConstMSRoutePtr& route, const SUMOTime currentTime, const double cost, const bool onInit) const;

private:
    /// @brief Invalidated copy constructor.
    MSDevice_Navi(const MSDevice_Navi&);

    /// @brief Invalidated assignment operator.
    MSDevice_Navi& operator=(const MSDevice_Navi&);
};

