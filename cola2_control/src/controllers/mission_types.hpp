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
    void show()
    {
        std::cout << *this;
    }

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

    void show()
    {
        std::cout << *this;
    }

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
        stream << "Action "<< a.action_id << ": with ";
        stream << a.parameters.size() << " params";
        //for (std::vector<std::string>::const_iterator i = a.parameters.begin(); i != a.parameters.end(); i++) {
        //    stream << *i << ", ";
        //}
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

    std::string to_string(double value)
    {
        std::ostringstream sstream;
        sstream << value;
        return sstream.str();
    }

    void addStep(MissionStep *step)
    {
        step->step_id = _step_size++;
        _mission.push_back(step);
    }

    int loadConfiguration(TiXmlHandle hDoc, MissionConfiguration &conf) {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        if (!pElem) return -3; // No key element
        std::string key_tag = pElem->Value();
        conf.key = pElem->GetText();
        pElem = pElem->NextSiblingElement();
        if (!pElem) return -4; // No value element
        std::string value_tag = pElem->Value();
        conf.value = pElem->GetText();
        if (key_tag == "key" && value_tag == "value") {
            std::cout << "Load: " << conf << std::endl;
        }
    }

    int loadAction(TiXmlHandle hDoc, MissionAction &action) {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        if (!pElem) return -5; // No action element
        std::string action_tag = pElem->Value();
        action.action_id = pElem->GetText();
        if (action_tag != "action_id") return -6; // Invalid action_id tag

        // Check params
        pElem = pElem=pElem->NextSiblingElement();
        if (!pElem || pElem->Value() == "parameters") return 0; // No parameters
        hDoc = TiXmlHandle(pElem);

        // TODO: Check that parameters are really stored!
        pElem = hDoc.FirstChild().Element();
        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string param_tag = pElem->Value();
            std::string param = pElem->GetText();
            if (param_tag == "param") {
                action.parameters.push_back(param);
            }
        }
         std::cout << "Load: " << action << std::endl;
    }

    bool loadPosition(TiXmlHandle hDoc, MissionPosition &position)
    {
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
                position.latitude = atof(pElem->GetText());
                lat = true;
            }
            else if (wp_tag == "longitude") {
                position.longitude = atof(pElem->GetText());
                lon = true;
            }
            else if (wp_tag == "z") {
                position.z = atof(pElem->GetText());
                z_ = true;
            }
            else if (wp_tag == "altitude_mode") {
                position.altitude_mode = pElem->GetText() == "true";
                mode = true;
            }
        }
        return (lat && lon && z_ && mode);
    }

    bool loadTolerance(TiXmlHandle hDoc, MissionTolerance &tolerance)
    {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        bool x_ = false;
        bool y_ = false;
        bool z_ = false;

        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string wp_tag = pElem->Value();
            if (wp_tag == "x") {
                tolerance.x = atof(pElem->GetText());
                x_ = true;
            }
            else if (wp_tag == "y") {
                tolerance.y = atof(pElem->GetText());
                y_ = true;
            }
            else if (wp_tag == "z") {
                tolerance.z = atof(pElem->GetText());
                z_ = true;
            }
            if (x_ && y_ && z_) break;
        }
        return (x_ && y_ && z_);
    }

    bool loadManeuverWaypoint(TiXmlHandle hDoc, MissionWaypoint &waypoint) {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        bool position = false;
        bool speed = false;
        bool tolerance = false;

        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string wp_tag = pElem->Value();
            if (wp_tag == "position") {
                position = loadPosition(TiXmlHandle(pElem), waypoint.position);
            }
            else if (wp_tag == "speed") {
                waypoint.speed = atof(pElem->GetText());
                speed = true;
            }
            else if (wp_tag == "tolerance") {
                tolerance = loadTolerance(TiXmlHandle(pElem), waypoint.tolerance);
            }
            if (position && speed && tolerance) break;
        }
        std::cout << "Load: " << waypoint << std::endl;
        return (position && speed && tolerance);
    }

    bool loadManeuverSection(TiXmlHandle hDoc, MissionSection &section) {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        bool initial_position = false;
        bool final_position = false;
        bool speed = false;
        bool tolerance = false;

        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string wp_tag = pElem->Value();
            if (wp_tag == "initial_position") {
                initial_position = loadPosition(TiXmlHandle(pElem), section.initial_position);
            }
            else if (wp_tag == "final_position") {
                final_position = loadPosition(TiXmlHandle(pElem), section.final_position);
            }
            else if (wp_tag == "speed") {
                section.speed = atof(pElem->GetText());
                speed = true;
            }
            else if (wp_tag == "tolerance") {
                tolerance = loadTolerance(TiXmlHandle(pElem), section.tolerance);
            }
            if (initial_position && final_position && speed && tolerance) break;
        }
        std::cout << "Load: " << section << std::endl;
        return (initial_position && final_position && speed && tolerance);
    }

    bool loadManeuverPark(TiXmlHandle hDoc, MissionPark &park) {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        bool position = false;
        bool time = false;
        bool tolerance = false;

        for(pElem; pElem; pElem=pElem->NextSiblingElement())
        {
            std::string wp_tag = pElem->Value();
            if (wp_tag == "position") {
                position = loadPosition(TiXmlHandle(pElem), park.position);
            }
            else if (wp_tag == "time") {
                park.time = atof(pElem->GetText());
                time = true;
            }
            else if (wp_tag == "tolerance") {
                tolerance = loadTolerance(TiXmlHandle(pElem), park.tolerance);
            }
            if (position && time && tolerance) break;
        }
        std::cout << "Load: " << park << std::endl;
        return (position && time && tolerance);
    }

    int loadMission(const std::string mission_file_name)
    {
        // Load XML document
        TiXmlDocument doc(mission_file_name.c_str());
	    if (!doc.LoadFile()) return -1; // Invalid/Not found document

        TiXmlHandle hDoc(&doc);
        TiXmlElement* pElem;

        // If previous mission erase it
        _mission.clear();

        // Read all childs of mission tag
        pElem = hDoc.FirstChild("mission").FirstChild().Element();
        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string m_name = pElem->Value();
            if (m_name == "configuration") {
                MissionConfiguration *conf = new MissionConfiguration();
                loadConfiguration(TiXmlHandle(pElem), *conf);
                addStep(conf);
            }
            else if (m_name == "action") {
                MissionAction *action = new MissionAction();
                loadAction(TiXmlHandle(pElem), *action);
                addStep(action);
            }
            else if (m_name == "maneuver") {
                std::string attribute = pElem->Attribute("type");
                if (attribute == "waypoint") {
                    MissionWaypoint *waypoint = new MissionWaypoint();
                    loadManeuverWaypoint(TiXmlHandle(pElem), *waypoint);
                    addStep(waypoint);
                }
                else if (attribute == "section") {
                    MissionSection *section = new MissionSection();
                    loadManeuverSection(TiXmlHandle(pElem), *section);
                    addStep(section);
                }
                else if (attribute == "park") {
                    MissionPark *park = new MissionPark();
                    loadManeuverPark(TiXmlHandle(pElem), *park);
                    addStep(park);
                }
                else {
                    std::cout << "Invalid maneuver type: " << attribute << std::endl;
                }
            }
        }


        return 0; // Everything ok
    }

    int writeConfiguration(TiXmlElement *mission, MissionConfiguration &conf)
    {
        std::cout << "Write: " << conf << std::endl;
        TiXmlElement * element = new TiXmlElement("configuration");
        TiXmlElement * key = new TiXmlElement("key");
        key->LinkEndChild(new TiXmlText(conf.key));
        element->LinkEndChild(key);
        TiXmlElement * value = new TiXmlElement("value");
        value->LinkEndChild(new TiXmlText(conf.value));
        element->LinkEndChild(value);
        mission->LinkEndChild(element);
    }

    int writeAction(TiXmlElement *mission, MissionAction &action)
    {
        std::cout << "Write: " << action << std::endl;
        TiXmlElement * element = new TiXmlElement("action");
        TiXmlElement * action_id = new TiXmlElement("action_id");
        action_id->LinkEndChild(new TiXmlText(action.action_id));
        element->LinkEndChild(action_id);
        if (action.parameters.size() > 0) {
            TiXmlElement * parameters = new TiXmlElement("parameters");
            element->LinkEndChild(parameters);
            for (std::vector<std::string>::iterator i = action.parameters.begin(); i != action.parameters.end(); i++) {
                TiXmlElement * param = new TiXmlElement("param");
                param->LinkEndChild(new TiXmlText(*i));
                parameters->LinkEndChild(param);
            }
        }
        mission->LinkEndChild(element);
    }

    int writeManeuverPosition(TiXmlElement *maneuver, MissionPosition &p, std::string position_tag)
    {
        TiXmlElement *position = new TiXmlElement(position_tag);
        maneuver->LinkEndChild(position);
        TiXmlElement *lat = new TiXmlElement("latitude");
        lat->LinkEndChild(new TiXmlText(to_string(p.latitude)));
        position->LinkEndChild(lat);
        TiXmlElement *lon = new TiXmlElement("longitude");
        lon->LinkEndChild(new TiXmlText(to_string(p.longitude)));
        position->LinkEndChild(lon);
        TiXmlElement *z = new TiXmlElement("z");
        z->LinkEndChild(new TiXmlText(to_string(p.z)));
        position->LinkEndChild(z);
        TiXmlElement *mode = new TiXmlElement("altitude_mode");
        if (p.altitude_mode) mode->LinkEndChild(new TiXmlText("true"));
        else mode->LinkEndChild(new TiXmlText("false"));
        position->LinkEndChild(mode);
        return 0;
    }

    int writeManeuverTolerance(TiXmlElement *maneuver, MissionTolerance &tol)
    {
        TiXmlElement *tolerance = new TiXmlElement("tolerance");
        maneuver->LinkEndChild(tolerance);
        TiXmlElement *x = new TiXmlElement("x");
        x->LinkEndChild(new TiXmlText(to_string(tol.x)));
        tolerance->LinkEndChild(x);
        TiXmlElement *y = new TiXmlElement("y");
        y->LinkEndChild(new TiXmlText(to_string(tol.y)));
        tolerance->LinkEndChild(y);
        TiXmlElement *z = new TiXmlElement("z");
        z->LinkEndChild(new TiXmlText(to_string(tol.z)));
        tolerance->LinkEndChild(z);
        return 0;
    }


    int writeManeuverWaypoint(TiXmlElement *mission, MissionWaypoint &wp)
    {
        std::cout << "Write: " << wp << std::endl;
        TiXmlElement * element = new TiXmlElement("maneuver");
        element->SetAttribute("type", "waypoint");
        mission->LinkEndChild(element);
        writeManeuverPosition(element, wp.position, "position");
        writeManeuverTolerance(element, wp.tolerance);
        TiXmlElement *speed = new TiXmlElement("speed");
        speed->LinkEndChild(new TiXmlText(to_string(wp.speed)));
        element->LinkEndChild(speed);
    }

    int writeManeuverSection(TiXmlElement *mission, MissionSection &sec)
    {
        std::cout << "Write: " << sec << std::endl;
        TiXmlElement * element = new TiXmlElement("maneuver");
        element->SetAttribute("type", "section");
        mission->LinkEndChild(element);
        writeManeuverPosition(element, sec.initial_position, "initial_position");
        writeManeuverPosition(element, sec.final_position, "final_position");
        writeManeuverTolerance(element, sec.tolerance);
        TiXmlElement *speed = new TiXmlElement("speed");
        speed->LinkEndChild(new TiXmlText(to_string(sec.speed)));
        element->LinkEndChild(speed);
    }

    int writeManeuverPark(TiXmlElement *mission, MissionPark &park)
    {
        std::cout << "Write: " << park << std::endl;
        TiXmlElement * element = new TiXmlElement("maneuver");
        element->SetAttribute("type", "park");
        mission->LinkEndChild(element);
        writeManeuverPosition(element, park.position, "position");
        writeManeuverTolerance(element, park.tolerance);
        TiXmlElement *time = new TiXmlElement("time");
        time->LinkEndChild(new TiXmlText(to_string(park.time)));
        element->LinkEndChild(time);
    }

    int writeMission(std::string mission_file_name)
    {
        TiXmlDocument doc;
        TiXmlElement* msg;
        TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
        doc.LinkEndChild(decl);
        TiXmlElement * mission = new TiXmlElement("mission");
        doc.LinkEndChild(mission);

        for (unsigned int i = 0; i < _mission.size(); i++) {
            MissionStep *step = _mission.at(i);
            if (step->step_type == MISSION_CONFIGURATION) {
                MissionConfiguration *conf = dynamic_cast<MissionConfiguration*>(step);
                writeConfiguration(mission, *conf);
            }
            else if (step->step_type == MISSION_ACTION) {
                MissionAction *act = dynamic_cast<MissionAction*>(step);
                writeAction(mission, *act);
            }
            else if (step->step_type == MISSION_MANEUVER) {
                MissionManeuver *m = dynamic_cast<MissionManeuver*>(step);
                if (m->getManeuverType() == WAYPOINT_MANEUVER) {
                    MissionWaypoint *wp = dynamic_cast<MissionWaypoint*>(m);
                    writeManeuverWaypoint(mission, *wp);
                }
                else if (m->getManeuverType() == SECTION_MANEUVER) {
                    MissionSection *sec = dynamic_cast<MissionSection*>(m);
                    writeManeuverSection(mission, *sec);
                }
                else if (m->getManeuverType() == PARK_MANEUVER) {
                    MissionPark *park = dynamic_cast<MissionPark*>(m);
                    writeManeuverPark(mission, *park);
                }
            }
        }
        doc.SaveFile(mission_file_name.c_str());
    }

private:
    unsigned int _step_size;
    std::vector<MissionStep*> _mission;
};


#endif // __MISSION_TYPES__
