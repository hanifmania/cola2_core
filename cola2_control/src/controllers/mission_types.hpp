#ifndef __MISSION_TYPES__
#define __MISSION_TYPES__

#include <vector>
#include <string>
#include <iostream>

#define WAYPOINT_MANEUVER       0
#define SECTION_MANEUVER        1
#define PARK_MANEUVER           2
#define MISSION_MANEUVER        0
#define MISSION_CONFIGURATION   1
#define MISSION_ACTION          2

class MissionStep {
public:
    MissionStep(unsigned int step_type_):
        step_type(step_type_)
    {}

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionStep& ms)
    {
        stream << ms;
    }

    virtual void show()
    {
        std::cout << "MissionStep: To be overrided" << std::endl;
    }

    unsigned int step_id;
    unsigned int step_type;
};

class MissionManeuver: public MissionStep {
public:
    MissionManeuver(unsigned int type):
        MissionStep(MISSION_MANEUVER),
        _maneuver_type(type)
    {}

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionManeuver& mm)
    {
        stream << mm;
    }

    virtual void show()
    {
        std::cout << "MissionManeuver To be overrided" << std::endl;
    }

private:
    unsigned int _maneuver_type;
};

class MissionPosition {
public:
    MissionPosition() {}
    MissionPosition(const MissionPosition &position)
    {
        latitude = position.latitude;
        longitude = position.longitude;
        z = position.z;
        altitude_mode = position.altitude_mode;
    }
    MissionPosition(double latitude_,
                    double longitude_,
                    double z_,
                    bool altitude_mode_)
    {
        latitude = latitude_;
        longitude = longitude_;
        z = z_;
        altitude_mode = altitude_mode_;
    }

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionPosition& pos)
    {
        stream << "[" << pos.latitude << ", " << pos.longitude << ", " << pos.z;
        if (pos.altitude_mode) stream << " (altitude)]";
        else stream << " (depth)]";
    }

    double latitude;
    double longitude;
    double z;
    bool altitude_mode;
};

class MissionTolerance {
public:
    MissionTolerance() {}
    MissionTolerance(const MissionTolerance &tolerance)
    {
        x = tolerance.x;
        y = tolerance.y;
        z = tolerance.z;
    }
    MissionTolerance(double x_, double y_, double z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionTolerance& tol)
    {
        stream << "[" << tol.x << ", " << tol.y << ", " << tol.z << "]";
    }

    double x;
    double y;
    double z;
};

class MissionWaypoint: public MissionManeuver {
public:
    MissionWaypoint():
        MissionManeuver(WAYPOINT_MANEUVER)
     {}

    MissionWaypoint(MissionPosition position_,
                    double speed_,
                    MissionTolerance tolerance_):
        MissionManeuver(WAYPOINT_MANEUVER),
        position(position_),
        speed(speed_),
        tolerance(tolerance_)
    {}

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionWaypoint& wp)
    {
        stream << "Waypoint -> " << wp.position << " at " << wp.speed << "m/s with tolerance " << wp.tolerance;
    }

    void show()
    {
        std::cout << *this;
    }

    MissionPosition position;
    double speed;
    MissionTolerance tolerance;
};

class MissionSection: public MissionManeuver {
public:
    MissionSection():
        MissionManeuver(SECTION_MANEUVER)
     {}

    MissionSection(MissionPosition initial_position_,
                   MissionPosition final_position_,
                   double speed_,
                   MissionTolerance tolerance_):
        MissionManeuver(SECTION_MANEUVER),
        initial_position(initial_position_),
        final_position(final_position_),
        speed(speed_),
        tolerance(tolerance_)
    {}

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionSection& s)
    {
        stream << "Section -> " << s.initial_position << " to " << s.final_position;
    }
    void show() {}

    MissionPosition initial_position;
    MissionPosition final_position;
    double speed;
    MissionTolerance tolerance;
};

class MissionPark: public MissionManeuver {
public:
    MissionPark():
        MissionManeuver(PARK_MANEUVER)
     {}

    MissionPark(MissionPosition position_,
                unsigned int time_,
                MissionTolerance tolerance_):
        MissionManeuver(PARK_MANEUVER),
        position(position_),
        time(time_),
        tolerance(tolerance_)
    {}

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionPark& p)
    {
        stream << "Park -> " << p.position << " for " << p.time << "s";
    }
    void show() {}

    MissionPosition position;
    unsigned int time;
    MissionTolerance tolerance;
};

class MissionConfiguration: public MissionStep {
public:
    MissionConfiguration():
        MissionStep(MISSION_CONFIGURATION)
    {}

    MissionConfiguration(std::string key_,
                         std::string value_):
        MissionStep(MISSION_CONFIGURATION),
        key(key_),
        value(value_)
    {}

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionConfiguration& conf)
    {
        stream << "configuration -> " << conf.key << ": " << conf.value;
    }

    void show()
    {
        std::cout << *this;
    }

    std::string key;
    std::string value;
};

class MissionAction: public MissionStep {
public:
    MissionAction():
        MissionStep(MISSION_ACTION)
    {}

    MissionAction(std::string action_id_,
                  std::vector<std::string> parameters_):
        MissionStep(MISSION_ACTION),
        action_id(action_id_),
        parameters(parameters_)
    {}

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionAction& a)
    {
        stream << "Action -> "<< a.action_id << ": ";
        for (std::vector<std::string>::const_iterator i = a.parameters.begin(); i != a.parameters.end(); i++) {
            stream << *i << ", ";
        }
    }

    void show()
    {
        std::cout << *this;
    }

    std::string action_id;
    std::vector<std::string> parameters;
};

class Mission {
public:
    Mission():
        _step_size(0)
    {}

    void show()
    {
        std::cout << "Mission steps: " << _mission.size() << std::endl;
        std::cout << "----------------------" << std::endl;
        for (unsigned int i = 0; i < _mission.size(); i++) {
            std::cout << "Step: " << _mission.at(i)->step_id << std::endl;
            _mission.at(i)->show();
            std::cout << std::endl << "----------------------" << std::endl;
        }
    }

    void addStep(MissionStep *step)
    {
        step->step_id = _step_size++;
        _mission.push_back(step);
    }

private:
    unsigned int _step_size;
    std::vector<MissionStep*> _mission;
};


#endif // __MISSION_TYPES__
