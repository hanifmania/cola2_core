#ifndef __MISSION_TYPES__
#define __MISSION_TYPES__

#include <vector>
#include <string>
#include <iostream>
#include "tinyxml.h"

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

    unsigned int getManeuverType()
    {
        return _maneuver_type;
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

    void extraMethod()
    {
        std::cout << "Yess!" << std::endl;
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

    MissionStep* getStep(unsigned int i)
    {
        assert(i < _mission.size());
        return _mission.at(i);
    }

    unsigned int size() {
        return _mission.size();
    }

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

    int loadConfiguration(TiXmlHandle hDoc) {
        TiXmlElement* pElem;
        std::cout << "Load configuration.." << std::endl;
        pElem = hDoc.FirstChild().Element();
        if (!pElem) return -3; // No key element
        std::string key_tag = pElem->Value();
        std::string key = pElem->GetText();
        pElem = pElem->NextSiblingElement();
        if (!pElem) return -4; // No value element
        std::string value_tag = pElem->Value();
        std::string value = pElem->GetText();
        if (key_tag == "key" && value_tag == "value") {
            std::cout << key << ": " << value << std::endl;
        }
    }

    int loadAction(TiXmlHandle hDoc) {
        TiXmlElement* pElem;
        std::cout << "Load action.." << std::endl;
        pElem = hDoc.FirstChild().Element();
        if (!pElem) return -5; // No action element
        std::string action_tag = pElem->Value();
        std::string action = pElem->GetText();
        if (action_tag != "action_id") return -6; // Invalid action_id tag
        std::cout << "action_id: " << action << std::endl;

        // Check params
        pElem = pElem=pElem->NextSiblingElement();
        if (!pElem || pElem->Value() == "parameters") return 0; // No parameters
        hDoc = TiXmlHandle(pElem);

        pElem = hDoc.FirstChild().Element();
        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string param_tag = pElem->Value();
            std::string param = pElem->GetText();
            if (param_tag == "param") {
                std::cout << "\t" << param << std::endl;
            }
        }
    }

    bool loadPosition(TiXmlHandle hDoc)
    {
        std::cout << "Load position" << std::endl;
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        bool lat = false;
        bool lon = false;
        bool z_ = false;
        bool mode = false;

        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string wp_tag = pElem->Value();
            if (wp_tag == "latitude") {
                double latitude = atof(pElem->GetText());
                std::cout << "latitude: " << latitude << std::endl;
                lat = true;
            }
            else if (wp_tag == "longitude") {
                double longitude = atof(pElem->GetText());
                std::cout << "longitude: " << longitude << std::endl;
                lon = true;
            }
            else if (wp_tag == "z") {
                double z = atof(pElem->GetText());
                std::cout << "z: " << z << std::endl;
                z_ = true;
            }
            else if (wp_tag == "altitude_mode") {
                bool altitude_mode = pElem->GetText() == "true";
                std::cout << "altitude_mode: " << altitude_mode << std::endl;
                mode = true;
            }
        }
        return (lat && lon && z_ && mode);
    }

    int loadManeuverWaypoint(TiXmlHandle hDoc) {
        TiXmlElement* pElem;
        std::cout << "Load waypoint maneuver.." << std::endl;
        pElem = hDoc.FirstChild().Element();
        bool position = false;
        bool speed = false;
        bool tolerance = false;
        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string wp_tag = pElem->Value();
            if (wp_tag == "position") {
                position = loadPosition(TiXmlHandle(pElem));
            }
            else if (wp_tag == "speed") {
                double speed_v = atof(pElem->GetText());
                std::cout << "Speed: " << speed_v << std::endl;
                speed = true;
            }
            else if (wp_tag == "tolerance") {
                std::cout << "Load tolerance" << std::endl;
            }
            if (position && speed && tolerance) break;
        }
    }

    int loadMission(const std::string mission_file_name)
    {
        // Load XML document
        TiXmlDocument doc(mission_file_name.c_str());
	    if (!doc.LoadFile()) return -1; // Invalid/Not found document

        TiXmlHandle hDoc(&doc);
        TiXmlElement* pElem;
        TiXmlHandle hRoot(0);

        // Read all childs of mission tag
        pElem = hDoc.FirstChild("mission").FirstChild().Element();
        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string m_name = pElem->Value();
            if (m_name == "configuration") {
                loadConfiguration(TiXmlHandle(pElem));
            }
            else if (m_name == "action") {
                loadAction(TiXmlHandle(pElem));
            }
            else if (m_name == "maneuver") {
                std::string attribute = pElem->Attribute("type");
                if (attribute == "waypoint") {
                    loadManeuverWaypoint(TiXmlHandle(pElem));
                }
                else {
                    std::cout << "Invalid maneuver type: " << attribute << std::endl;
                }
            }
        }

        return 0; // Everything ok
    }

private:
    unsigned int _step_size;
    std::vector<MissionStep*> _mission;
};


#endif // __MISSION_TYPES__
