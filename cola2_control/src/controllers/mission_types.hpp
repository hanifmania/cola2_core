
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

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


class MissionManeuver
{
public:
    MissionManeuver(const unsigned int type):
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

	virtual double
	x()
	{
		return 0.0;
	}

	virtual double
	y()
	{
		return 0.0;
	}

	virtual double
	z()
	{
		return 0.0;
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

    double
    x()
    {
        return position.latitude;
    }

    double
    y()
    {
        return position.longitude;
    }

    double
    z()
    {
        return position.z;
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

    double
    x()
    {
        return final_position.latitude;
    }

    double
    y()
    {
        return final_position.longitude;
    }

    double
    z()
    {
        return final_position.z;
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

    double
    x()
    {
        return position.latitude;
    }

    double
    y()
    {
        return position.longitude;
    }

    double
    z()
    {
        return position.z;
    }
    
    MissionPosition position;
    unsigned int time;
    MissionTolerance tolerance;
};

class MissionConfiguration
{
public:
    MissionConfiguration()
    {}

    MissionConfiguration(std::string key_,
                         std::string value_):
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

class MissionAction
{
public:
    MissionAction()
    {}

    MissionAction(std::string action_id_,
                  std::vector<std::string> parameters_):
        action_id(action_id_),
        parameters(parameters_),
        _is_empty(true)
    {
        if (parameters_.size() > 0) _is_empty = false;
    }

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionAction& a)
    {
        stream << "Action "<< a.action_id;
        if (!a._is_empty) {
            stream << ": with " << a.parameters.size() << " params: ";
            for (std::vector<std::string>::const_iterator i = a.parameters.begin(); i != a.parameters.end(); i++) {
                stream << *i << ", ";
            }
        }
    }

    void show()
    {
        std::cout << *this;
    }

    std::string action_id;
    std::vector<std::string> parameters;
    bool _is_empty;
};


class MissionStep
{
public:
    MissionStep()
    {}

    friend std::ostream& operator<< (std::ostream& stream,
                                     const MissionStep& ms)
    {
        stream << "????\n";
    }

    void show()
    {
        std::cout << "Maneuver:\n";
        maneuver_->show();
        std::cout << std::endl;
        std::cout << "List of actions:\n";
        for (std::vector<MissionAction>::iterator action = actions_.begin(); action != actions_.end(); ++action)
        {
            action->show();
            std::cout << std::endl;
        }
        // std::cout << *this;
    }

	MissionManeuver*
	getManeuver() const
	{
		return maneuver_;
	}

    std::vector<MissionAction>
    getActions() const
    {
        return actions_;
    }

    void
    setManeuver(MissionManeuver* maneuver)
    {
        maneuver_ = maneuver;
    }

    void addAction(MissionAction action)
    {
        actions_.push_back(action);
    }

    unsigned int step_id;
	MissionManeuver* maneuver_;
	std::vector<MissionAction> actions_;
};


class Mission {
public:
    Mission():
        _step_size(0)
    {}

    MissionStep*
    getStep(unsigned int i)
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

    /*
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
    */

    int
    loadAction(TiXmlHandle hDoc, MissionAction& action) {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        if (!pElem)
        {
            std::cerr << "No action element.\n";
            return -5; // No action element
        }
        std::string action_tag = pElem->Value();
        if (action_tag != "action_id")
        {
            std::cerr << "Error: action_id expected.\n";
            return -6; // Invalid action_id tag
        }
        action.action_id = pElem->GetText();


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
        //  std::cout << "Load: " << action << std::endl;
    }

    bool
    loadPosition(TiXmlHandle hDoc, MissionPosition &position)
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

    bool
    loadTolerance(TiXmlHandle hDoc, MissionTolerance &tolerance)
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

    bool
    loadManeuverWaypoint(TiXmlHandle hDoc, MissionWaypoint &waypoint) {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild().Element();
        bool position = false;
        bool speed = false;
        bool tolerance = false;
        std::cout << "Load maneuver waypoint\n";
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
        // std::cout << "Loaded!" << std::endl;
        return (position && speed && tolerance);
    }

    bool
    loadManeuverSection(TiXmlHandle hDoc, MissionSection &section) {
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
        // std::cout << "Loadede " << section << std::endl;
        return (initial_position && final_position && speed && tolerance);
    }

    bool
    loadManeuverPark(TiXmlHandle hDoc, MissionPark &park) {
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
        // std::cout << "Load: " << park << std::endl;
        return (position && time && tolerance);
    }

    int
    loadStep(TiXmlHandle hDoc, MissionStep &step)
    {
        TiXmlElement* pElem;
        pElem = hDoc.FirstChild("maneuver").Element();
        if (!pElem)
        {
            std::cout << "No maneuver element in mission step!\n";
            return -3; // No key element
        }
        else
        {
            std::string attribute = pElem->Attribute("type");
            if (attribute == "waypoint") {
                std::cout << "Waypoint maneuver found " << std::endl;
                MissionWaypoint *waypoint = new MissionWaypoint();
                loadManeuverWaypoint(TiXmlHandle(pElem), *waypoint);
                step.setManeuver(waypoint);
            }
            else if (attribute == "section") {
                MissionSection *section = new MissionSection();
                loadManeuverSection(TiXmlHandle(pElem), *section);
                step.setManeuver(section);
            }
            else if (attribute == "park") {
                MissionPark *park = new MissionPark();
                loadManeuverPark(TiXmlHandle(pElem), *park);
                step.setManeuver(park);
            }
            else {
                std::cout << "Invalid maneuver type: " << attribute << std::endl;
            }
        }

        pElem = hDoc.FirstChild("actions_list").FirstChild().Element();
        if (!pElem)
        {
            std::cout << "No actions found in mission step!\n";
        }
        else
        {
            std::cout << "Actions found in mission step!\n";
            for(pElem; pElem; pElem=pElem->NextSiblingElement())
    		{
                std::string m_name = pElem->Value();
                std::cout << "found: " << m_name << std::endl;
                if (m_name == "action")
                {
                    std::cout << "Load action ...\n";
                    MissionAction action;
                    loadAction(TiXmlHandle(pElem), action);
                    step.addAction(action);
                }
            }
        }
        return 0; // Everything ok
    }

    int
    loadMission(const std::string mission_file_name)
    {
        std::cout << "Load mission " << mission_file_name << std::endl;

        // Load XML document
        TiXmlDocument doc(mission_file_name.c_str());
	    if (!doc.LoadFile())
        {
            std::cout << "Invalid/Not found document.\n";
            return -1; // Invalid/Not found document
        }

        TiXmlHandle hDoc(&doc);
        TiXmlElement* pElem;

        // If previous mission erase it
        _mission.clear();

        // Read all childs of mission tag
        pElem = hDoc.FirstChild("mission").FirstChild().Element();
        std::string m_name = pElem->Value();
        for(pElem; pElem; pElem=pElem->NextSiblingElement())
		{
            std::string m_name = pElem->Value();
            if (m_name == "mission_step")
            {
                std::cout << "Mission step found " << std::endl;
                MissionStep *step = new MissionStep();
                loadStep(TiXmlHandle(pElem), *step);
                addStep(step);
            }
            else
            {
                std::cout << "Error readind mission step. Found " << m_name << ".\n";
            }
        }
        return 0;
    }

    /*
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
    */

    int
    writeAction(TiXmlElement *mission, MissionAction &action)
    {
        std::cout << "Write: " << action.action_id << std::endl;

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
        return 0;
    }

    int
    writeManeuverPosition(TiXmlElement *maneuver, MissionPosition &p, std::string position_tag)
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

    int
    writeManeuverTolerance(TiXmlElement *maneuver, MissionTolerance &tol)
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

    int
    writeManeuverWaypoint(TiXmlElement *mission, MissionWaypoint &wp)
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
        return 0;
    }

    int
    writeManeuverSection(TiXmlElement *mission, MissionSection &sec)
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
        return 0;
    }

    int
    writeManeuverPark(TiXmlElement *mission, MissionPark &park)
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
        return 0;
    }

    int
    writeMissionStep(TiXmlElement *mission, MissionStep &step)
    {
        std::cout << "Write: " << step << std::endl;
        TiXmlElement * mission_step = new TiXmlElement("mission_step");

        // Write mission step maneuver
        if (step.getManeuver()->getManeuverType() == WAYPOINT_MANEUVER)
        {
            std::cout << "Add waypoint\n";
            MissionWaypoint *wp = dynamic_cast<MissionWaypoint*>(step.getManeuver());
            writeManeuverWaypoint(mission_step, *wp);
        }
        else if (step.getManeuver()->getManeuverType() == SECTION_MANEUVER)
        {
            MissionSection *sec = dynamic_cast<MissionSection*>(step.getManeuver());
            writeManeuverSection(mission_step, *sec);
        }
        else if (step.getManeuver()->getManeuverType() == PARK_MANEUVER)
        {
            MissionPark *park = dynamic_cast<MissionPark*>(step.getManeuver());
            writeManeuverPark(mission_step, *park);
        }

        // Write action_list if available
        std::vector<MissionAction> actions = step.getActions();
        if (actions.size() > 0)
        {
            TiXmlElement * action_list = new TiXmlElement("actions_list");
            for (std::vector<MissionAction>::iterator action = actions.begin(); action != actions.end(); ++action)
            {
                action->show();
                writeAction(action_list, *action);
            }
            mission_step->LinkEndChild(action_list);
        }
        mission->LinkEndChild(mission_step);

        return 0;
    }

    int
    writeMission(std::string mission_file_name)
    {
        TiXmlDocument doc;
        TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
        doc.LinkEndChild(decl);
        TiXmlElement * mission = new TiXmlElement("mission");
        doc.LinkEndChild(mission);

        for (unsigned int i = 0; i < _mission.size(); i++)
        {
            MissionStep *step = _mission.at(i);
            writeMissionStep(mission, *step);
        }
        doc.SaveFile(mission_file_name.c_str());
        return 0;
    }

private:
    unsigned int _step_size;
    std::vector<MissionStep*> _mission;
};

#endif // __MISSION_TYPES__
