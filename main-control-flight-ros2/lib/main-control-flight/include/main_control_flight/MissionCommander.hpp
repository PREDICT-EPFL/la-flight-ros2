#ifndef SRC_MISSIONCOMMANDER_HPP
#define SRC_MISSIONCOMMANDER_HPP

#include <vector>
#include <map>
#include <fstream>

#include "fixedwing/FixedwingControl.hpp"
#include "main_control_common/ControlTypes.hpp"

namespace control {
namespace flight {
namespace fixedwing {

class ModeStringMapper
{
public:
    explicit ModeStringMapper(std::vector<std::pair<int, std::string>> vector_) : vector(std::move(vector_))
    {
        /* Initialize map from vector */
        for (const auto &entry: vector)
        {
            if (entry.second.at(0) != '_')
                map.emplace(entry.first, entry.second);
        }
    }
    bool contains(const int &mode)
    {
        return map.find(mode) != map.end();
    }
    bool getString(const int &mode, std::string &name)
    {
        if (contains(mode))
        {
            name = map.at(mode);
            return true;
        }
        else
        {
            name = "ERROR_MODE_DOES_NOT_EXIST";
            return false;
        }
    }
    std::string getString(const int &mode)
    {
        if (contains(mode))
        {
            return map.at(mode);
        }
        else
        {
            return "ERROR_MODE_DOES_NOT_EXIST";
        }
    }
    void getOptions(std::vector<std::pair<std::string, int>> &options)
    {
        for (const auto &entry: vector)
        {
            options.emplace_back(entry.second, entry.first);
        }
    }
    std::vector<std::pair<std::string, int>> getOptions()
    {
        std::vector<std::pair<std::string, int>> options;
        getOptions(options);
        return options;
    }
private:
    std::vector<std::pair<int, std::string>> vector;
    std::map<int, std::string> map;
};

enum MissionMode
{
    NOMISSION = 0,

    MAWE = 10,
};
class MissionModeStringMapper : public ModeStringMapper
{
public:
    MissionModeStringMapper() :
            ModeStringMapper({
                                     {NOMISSION,      "NOMISSION"},
                                     {0,              "_empty_line"},

                                     {MAWE,           "MULTI_AWE"},
                             }) {}
} static missionModeStringMapper;

class MissionCommander
{
public:
    MissionCommander()
    {
        t_mission_start = SteadyClock::now();
        t_leg_start = t_mission_start;
    }

    void getMissionName(std::string &name) const { name = missionName; }
    void setMission(const MissionMode &missionMode, const std::string &name)
    {
        nextMission = missionMode;
        nextMissionName = name;

        if (!mission_isActive)
        {
//            reset();
//            mission = missionMode;
//            missionName = name;
        }
    }
    void set(const std::string &missionMode)
    {
        if (missionMode == "mawe") { setMission(MAWE, missionModeStringMapper.getString(MAWE)); }
        else if (missionMode.empty())
        {
            setMission(MissionMode::NOMISSION, missionModeStringMapper.getString(NOMISSION));
        }
        else
        {
            std::cerr << "Mission mode """ << missionMode << """ could not be found. Falling back to NOMISSION.\n"
                      << "Available modes are: {mawe, }\n";
        }
    }
    MissionMode getMission() const
    {
        if (isActive())
            return mission;
        else
            return nextMission;
    }

    bool isActive() const { return mission_isActive; }
    int getActiveLegId()
    {
        if (legChange_isRequired) { changeLeg(); }
        return activeLegId;
    }
    int getMissionPhase() const { return activeLegId; }
    double getMissionAge() const
    {
        using namespace std::chrono;
        const unsigned age_us = duration_cast<microseconds>(SteadyClock::now() - t_mission_start).count();
        return age_us / static_cast<double>(1e6);
    }
    double getLegAge() const
    {
        using namespace std::chrono;
        const unsigned age_us = duration_cast<microseconds>(SteadyClock::now() - t_leg_start).count();
        return age_us / static_cast<double>(1e6);
    }

    void reportTaskDone() { leg_isDone = true; }

    void activateMarker() { marker_isActive = true; }
    void deactivateMarker() { marker_isActive = false; }
    bool getMarkerActive() const { return marker_isActive; }

    void startMission()
    {
        mission = nextMission;
        missionName = nextMissionName;
        reset();
        printStart();
        deactivateMarker();
        requestNextLeg();
        t_mission_start = SteadyClock::now();
        mission_isActive = true;
        mission_gotActive = true;
    }
    bool missionStarted()
    {
        return callbackFlag(mission_gotActive);
    }

    bool doOnce()
    {
        requestNextLeg();
        return callbackFlag(leg_hasChanged);
    }
    bool doUntilDone()
    {
        if (leg_isDone) { requestNextLeg(); }
        return callbackFlag(leg_hasChanged);
    }
    bool doFor_Seconds(const double &seconds)
    {
        if (getLegAge() >= seconds) { requestNextLeg(); }
        return callbackFlag(leg_hasChanged);
    }

    bool endMission()
    {
        deactivateMarker();
        mission_isActive = false;
        mission_gotInactive = true;
        printEnd();
        requestNextLeg();
        return callbackFlag(leg_hasChanged);
    }
    bool missionEnded() { return callbackFlag(mission_gotInactive); }

    bool runAgain() { return callbackFlag(reselect); }

private:
    MissionMode mission{MissionMode::NOMISSION};
    MissionMode nextMission{MissionMode::NOMISSION};
    std::string missionName = "NoMission";
    std::string nextMissionName = "NoMission";
    bool mission_isActive{false};
    bool mission_gotActive{false};
    bool mission_gotInactive{false};

    int activeLegId{-1};
    bool leg_isDone{false};
    bool legChange_isRequired{false};
    bool leg_hasChanged{false};
    bool reselect{false};

    void reset()
    {
        mission_isActive = false;
        activeLegId = -1;
        leg_isDone = false;
        legChange_isRequired = false;
        leg_hasChanged = false;
        deactivateMarker();
    }

    void requestNextLeg()
    {
        legChange_isRequired = true;
        reselect = true;
    }
    void changeLeg()
    {
        legChange_isRequired = false;
        ++activeLegId;
        leg_isDone = false;
        t_leg_start = SteadyClock::now();
        leg_hasChanged = true;
    }
    static bool callbackFlag(bool &flag)
    {
        if (flag)
        {
            flag = false;
            return true;
        }
        else
        {
            return false;
        }
    }

    /* Timers */
    TimePoint t_mission_start{};
    TimePoint t_leg_start{};

    /* Boolean to mark special parts of the mission */
    bool marker_isActive{false};

    void printStart() { std::cout << "[   MISSION   ]: " << missionName << "\n"; }
    void printEnd() { std::cout << "[ MISSION END ]: " << missionName << "\n"; }
};

} // namespace fixedwing
} // namespace flight
} // namespace control

#endif //SRC_MISSIONCOMMANDER_HPP