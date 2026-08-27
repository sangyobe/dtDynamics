/*!
 \file      dtDynSpec.cpp
 \brief     Editable native dtDynamics model specification definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.0
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynSpec.h"

#include <cctype>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace dt::Dynamics
{
namespace
{

using AttributeMap = std::unordered_map<std::string, std::string>;

static constexpr dtDynReal kPi = static_cast<dtDynReal>(3.141592653589793238462643383279502884);
static constexpr dtDynReal kDegreeToRadian = kPi / static_cast<dtDynReal>(180.0);

std::string Trim(const std::string &text)
{
    std::size_t first = 0;
    while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first])))
    {
        ++first;
    }

    std::size_t last = text.size();
    while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1])))
    {
        --last;
    }
    return text.substr(first, last - first);
}

std::string TagName(const std::string &tag)
{
    std::size_t pos = 0;
    while (pos < tag.size() && !std::isspace(static_cast<unsigned char>(tag[pos])) &&
           tag[pos] != '/')
    {
        ++pos;
    }
    return tag.substr(0, pos);
}

AttributeMap ParseAttributes(const std::string &tag)
{
    AttributeMap attrs;
    std::size_t pos = TagName(tag).size();
    while (pos < tag.size())
    {
        while (pos < tag.size() && std::isspace(static_cast<unsigned char>(tag[pos])))
        {
            ++pos;
        }
        if (pos >= tag.size() || tag[pos] == '/')
        {
            break;
        }

        const std::size_t keyStart = pos;
        while (pos < tag.size() && tag[pos] != '=' &&
               !std::isspace(static_cast<unsigned char>(tag[pos])))
        {
            ++pos;
        }
        const std::string key = tag.substr(keyStart, pos - keyStart);

        while (pos < tag.size() && std::isspace(static_cast<unsigned char>(tag[pos])))
        {
            ++pos;
        }
        if (pos >= tag.size() || tag[pos] != '=')
        {
            break;
        }
        ++pos;
        while (pos < tag.size() && std::isspace(static_cast<unsigned char>(tag[pos])))
        {
            ++pos;
        }
        if (pos >= tag.size() || (tag[pos] != '"' && tag[pos] != '\''))
        {
            break;
        }

        const char quote = tag[pos++];
        const std::size_t valueStart = pos;
        while (pos < tag.size() && tag[pos] != quote)
        {
            ++pos;
        }
        attrs[key] = tag.substr(valueStart, pos - valueStart);
        if (pos < tag.size())
        {
            ++pos;
        }
    }
    return attrs;
}

bool HasAttribute(const AttributeMap &attrs, const std::string &key)
{
    return attrs.find(key) != attrs.end();
}

std::string AttributeOr(const AttributeMap &attrs, const std::string &key,
                        const std::string &fallback)
{
    const auto it = attrs.find(key);
    return it == attrs.end() ? fallback : it->second;
}

dtDynReal RealAttributeOr(const AttributeMap &attrs, const std::string &key,
                          const dtDynReal fallback)
{
    const auto it = attrs.find(key);
    if (it == attrs.end())
    {
        return fallback;
    }
    return static_cast<dtDynReal>(std::stod(it->second));
}

dt::Math::Vector3<dtDynReal> ParseVector3(const std::string &text)
{
    dt::Math::Vector3<dtDynReal> value;
    value(0) = 0.0;
    value(1) = 0.0;
    value(2) = 0.0;

    std::istringstream stream(text);
    stream >> value(0) >> value(1) >> value(2);
    return value;
}

dt::Math::Vector4<dtDynReal> ParseVector4(const std::string &text)
{
    dt::Math::Vector4<dtDynReal> value;
    value(0) = 0.0;
    value(1) = 0.0;
    value(2) = 0.0;
    value(3) = 0.0;

    std::istringstream stream(text);
    stream >> value(0) >> value(1) >> value(2) >> value(3);
    return value;
}

std::string FormatReal(const dtDynReal value)
{
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    return stream.str();
}

std::string FormatVector3(const dt::Math::Vector3<dtDynReal> &value)
{
    return FormatReal(value(0)) + " " + FormatReal(value(1)) + " " + FormatReal(value(2));
}

std::string FormatVector4(const dt::Math::Vector4<dtDynReal> &value)
{
    return FormatReal(value(0)) + " " + FormatReal(value(1)) + " " + FormatReal(value(2)) + " " +
           FormatReal(value(3));
}

void SetIdentityQuat(dt::Math::Vector4<dtDynReal> &quat)
{
    quat(0) = 1.0;
    quat(1) = 0.0;
    quat(2) = 0.0;
    quat(3) = 0.0;
}

dtDynJointType JointTypeFromXml(const std::string &type)
{
    if (type == "free")
    {
        return dtDynJointType::Free;
    }
    if (type == "ball")
    {
        return dtDynJointType::Ball;
    }
    if (type == "slide")
    {
        return dtDynJointType::Slide;
    }
    return dtDynJointType::Hinge;
}

std::string JointTypeToXml(const dtDynJointType type)
{
    if (type == dtDynJointType::Free)
    {
        return "free";
    }
    if (type == dtDynJointType::Ball)
    {
        return "ball";
    }
    if (type == dtDynJointType::Slide)
    {
        return "slide";
    }
    return "hinge";
}

dtDynReal AngleScaleFromCompilerAttribute(const AttributeMap &attrs)
{
    const std::string angle = AttributeOr(attrs, "angle", "degree");
    if (angle == "radian")
    {
        return 1.0;
    }
    if (angle == "degree")
    {
        return kDegreeToRadian;
    }

    throw std::runtime_error("dtDynSpec: unsupported compiler angle unit");
}

void EncodeBody(const dtDynSpec &spec, const dtDynIndex bodyId, const dtDynIndex depth,
                std::ostringstream &out)
{
    const dtDynBodySpec &body = spec.Bodies()[static_cast<std::size_t>(bodyId)];
    const std::string indent(static_cast<std::size_t>(depth * 2), ' ');
    out << indent << "<body name=\"" << body.name << "\" pos=\"" << FormatVector3(body.pos) << "\"";
    if (body.quat(0) != 0.0 || body.quat(1) != 0.0 || body.quat(2) != 0.0 || body.quat(3) != 0.0)
    {
        out << " quat=\"" << FormatVector4(body.quat) << "\"";
    }
    out << ">\n";

    if (body.inertial.mass != 0.0)
    {
        out << indent << "  <inertial pos=\"" << FormatVector3(body.inertial.pos) << "\" mass=\""
            << FormatReal(body.inertial.mass) << "\" diaginertia=\""
            << FormatVector3(body.inertial.inertia) << "\"";
        if (body.inertial.quat(0) != 0.0 || body.inertial.quat(1) != 0.0 ||
            body.inertial.quat(2) != 0.0 || body.inertial.quat(3) != 0.0)
        {
            out << " quat=\"" << FormatVector4(body.inertial.quat) << "\"";
        }
        out << "/>\n";
    }

    for (const dtDynIndex jointId : body.joints)
    {
        const dtDynJointSpec &joint = spec.Joints()[static_cast<std::size_t>(jointId)];
        out << indent << "  <joint name=\"" << joint.name << "\" type=\""
            << JointTypeToXml(joint.type) << "\" pos=\"" << FormatVector3(joint.pos) << "\" axis=\""
            << FormatVector3(joint.axis) << "\" armature=\"" << FormatReal(joint.armature) << "\"";
        if (joint.limited)
        {
            out << " limited=\"true\" range=\"" << FormatReal(joint.rangeMin) << " "
                << FormatReal(joint.rangeMax) << "\"";
        }
        out << "/>\n";
    }

    for (const dtDynIndex siteId : body.sites)
    {
        const dtDynSiteSpec &site = spec.Sites()[static_cast<std::size_t>(siteId)];
        out << indent << "  <site name=\"" << site.name << "\" pos=\"" << FormatVector3(site.pos)
            << "\"";
        if (site.quat(0) != 0.0 || site.quat(1) != 0.0 || site.quat(2) != 0.0 ||
            site.quat(3) != 0.0)
        {
            out << " quat=\"" << FormatVector4(site.quat) << "\"";
        }
        out << "/>\n";
    }

    for (dtDynIndex child = body.firstChild; child >= 0;
         child = spec.Bodies()[static_cast<std::size_t>(child)].nextSibling)
    {
        EncodeBody(spec, child, depth + 1, out);
    }

    out << indent << "</body>\n";
}

} // namespace

dtDynSpec::dtDynSpec() { Clear(); }

dtDynIndex dtDynSpec::AddBody(const std::string &name, const dtDynIndex parent)
{
    dtDynBodySpec body;
    body.name = name;
    body.parent = parent;
    SetIdentityQuat(body.quat);
    SetIdentityQuat(body.inertial.quat);

    const dtDynIndex id = static_cast<dtDynIndex>(m_bodies.size());
    m_bodies.push_back(body);

    if (parent >= 0 && parent < id)
    {
        m_bodies[id].nextSibling = m_bodies[parent].firstChild;
        m_bodies[parent].firstChild = id;
    }

    return id;
}

dtDynIndex dtDynSpec::AddJoint(const dtDynIndex body, const std::string &name,
                               const dtDynJointType type)
{
    dtDynJointSpec joint;
    joint.name = name;
    joint.body = body;
    joint.type = type;
    joint.axis(0) = 0.0;
    joint.axis(1) = 0.0;
    joint.axis(2) = 1.0;

    const dtDynIndex id = static_cast<dtDynIndex>(m_joints.size());
    m_joints.push_back(joint);

    if (body >= 0 && body < static_cast<dtDynIndex>(m_bodies.size()))
    {
        m_bodies[body].joints.push_back(id);
    }

    return id;
}

dtDynIndex dtDynSpec::AddSite(const dtDynIndex body, const std::string &name)
{
    dtDynSiteSpec site;
    site.name = name;
    site.body = body;
    SetIdentityQuat(site.quat);

    const dtDynIndex id = static_cast<dtDynIndex>(m_sites.size());
    m_sites.push_back(site);

    if (body >= 0 && body < static_cast<dtDynIndex>(m_bodies.size()))
    {
        m_bodies[body].sites.push_back(id);
    }

    return id;
}

dtDynIndex dtDynSpec::AddActuator(const dtDynIndex joint, const std::string &name)
{
    dtDynActuatorSpec actuator;
    actuator.name = name;
    actuator.joint = joint;

    const dtDynIndex id = static_cast<dtDynIndex>(m_actuators.size());
    m_actuators.push_back(actuator);
    return id;
}

void dtDynSpec::Clear()
{
    m_bodies.clear();
    m_joints.clear();
    m_sites.clear();
    m_actuators.clear();
    AddBody("world", DTDYN_INVALID_INDEX);
}

std::string dtDynSpec::EncodeXml() const
{
    std::ostringstream out;
    out << "<mujoco>\n";
    out << "  <compiler angle=\"radian\"/>\n";
    out << "  <worldbody>\n";
    for (dtDynIndex child = m_bodies[0].firstChild; child >= 0;
         child = m_bodies[static_cast<std::size_t>(child)].nextSibling)
    {
        EncodeBody(*this, child, 2, out);
    }
    out << "  </worldbody>\n";
    if (!m_actuators.empty())
    {
        out << "  <actuator>\n";
        for (const dtDynActuatorSpec &actuator : m_actuators)
        {
            out << "    <motor name=\"" << actuator.name << "\"";
            if (actuator.joint >= 0 && actuator.joint < static_cast<dtDynIndex>(m_joints.size()))
            {
                out << " joint=\"" << m_joints[static_cast<std::size_t>(actuator.joint)].name
                    << "\"";
            }
            out << " gear=\"" << FormatReal(actuator.gear) << "\"/>\n";
        }
        out << "  </actuator>\n";
    }
    out << "</mujoco>\n";
    return out.str();
}

dtDynSpec dtDynSpec::DecodeXml(const std::string &xml)
{
    if (xml.find("<equality") != std::string::npos || xml.find("<tendon") != std::string::npos)
    {
        throw std::runtime_error(
            "dtDynSpec: equality constraints are not supported in the tree-only dynamics phase");
    }

    dtDynSpec spec;
    bool inWorldbody = false;
    bool inActuator = false;
    dtDynReal angleScale = kDegreeToRadian;
    dtArray<dtDynIndex> bodyStack;
    bodyStack.push_back(0);
    std::unordered_map<std::string, dtDynIndex> jointNameToId;

    std::size_t cursor = 0;
    while (true)
    {
        const std::size_t begin = xml.find('<', cursor);
        if (begin == std::string::npos)
        {
            break;
        }
        if (xml.compare(begin, 4, "<!--") == 0)
        {
            const std::size_t commentEnd = xml.find("-->", begin + 4);
            if (commentEnd == std::string::npos)
            {
                throw std::runtime_error("dtDynSpec: malformed XML comment");
            }
            cursor = commentEnd + 3;
            continue;
        }
        const std::size_t end = xml.find('>', begin + 1);
        if (end == std::string::npos)
        {
            throw std::runtime_error("dtDynSpec: malformed XML tag");
        }

        std::string tag = Trim(xml.substr(begin + 1, end - begin - 1));
        cursor = end + 1;
        if (tag.empty() || tag[0] == '?' || tag[0] == '!')
        {
            continue;
        }

        bool closing = false;
        if (tag[0] == '/')
        {
            closing = true;
            tag = Trim(tag.substr(1));
        }

        bool selfClosing = false;
        if (!tag.empty() && tag.back() == '/')
        {
            selfClosing = true;
            tag = Trim(tag.substr(0, tag.size() - 1));
        }

        const std::string name = TagName(tag);
        const AttributeMap attrs = ParseAttributes(tag);

        if (closing)
        {
            if (name == "worldbody")
            {
                inWorldbody = false;
            }
            else if (name == "actuator")
            {
                inActuator = false;
            }
            else if (name == "body" && inWorldbody && bodyStack.size() > 1)
            {
                bodyStack.resize(bodyStack.size() - 1);
            }
            continue;
        }

        if (name == "worldbody")
        {
            inWorldbody = true;
            continue;
        }
        if (name == "compiler")
        {
            angleScale = AngleScaleFromCompilerAttribute(attrs);
            continue;
        }
        if (name == "actuator")
        {
            inActuator = true;
            continue;
        }

        if (inWorldbody && name == "body")
        {
            const dtDynIndex parent = bodyStack[bodyStack.size() - 1];
            const dtDynIndex bodyId = spec.AddBody(AttributeOr(attrs, "name", "body"), parent);
            if (HasAttribute(attrs, "pos"))
            {
                spec.Bodies()[bodyId].pos = ParseVector3(attrs.at("pos"));
            }
            if (HasAttribute(attrs, "quat"))
            {
                spec.Bodies()[bodyId].quat = ParseVector4(attrs.at("quat"));
            }
            if (!selfClosing)
            {
                bodyStack.push_back(bodyId);
            }
            continue;
        }

        if (inWorldbody && name == "inertial" && bodyStack.size() > 1)
        {
            dtDynInertialSpec &inertial = spec.Bodies()[bodyStack[bodyStack.size() - 1]].inertial;
            inertial.mass = RealAttributeOr(attrs, "mass", inertial.mass);
            if (HasAttribute(attrs, "pos"))
            {
                inertial.pos = ParseVector3(attrs.at("pos"));
            }
            if (HasAttribute(attrs, "quat"))
            {
                inertial.quat = ParseVector4(attrs.at("quat"));
            }
            if (HasAttribute(attrs, "diaginertia"))
            {
                inertial.inertia = ParseVector3(attrs.at("diaginertia"));
            }
            continue;
        }

        if (inWorldbody && name == "joint" && bodyStack.size() > 1)
        {
            const dtDynIndex bodyId = bodyStack[bodyStack.size() - 1];
            const std::string jointName = AttributeOr(attrs, "name", "joint");
            const dtDynIndex jointId = spec.AddJoint(
                bodyId, jointName, JointTypeFromXml(AttributeOr(attrs, "type", "hinge")));
            dtDynJointSpec &joint = spec.Joints()[jointId];
            if (HasAttribute(attrs, "pos"))
            {
                joint.pos = ParseVector3(attrs.at("pos"));
            }
            if (HasAttribute(attrs, "axis"))
            {
                joint.axis = ParseVector3(attrs.at("axis"));
            }
            joint.armature = RealAttributeOr(attrs, "armature", joint.armature);
            const bool hasLimitedAttribute = HasAttribute(attrs, "limited");
            if (hasLimitedAttribute)
            {
                const std::string limited = attrs.at("limited");
                joint.limited = limited == "true" || limited == "1";
            }
            if (HasAttribute(attrs, "range"))
            {
                std::istringstream stream(attrs.at("range"));
                stream >> joint.rangeMin >> joint.rangeMax;
                joint.rangeMin *= angleScale;
                joint.rangeMax *= angleScale;
                if (!hasLimitedAttribute)
                {
                    joint.limited = true;
                }
            }
            jointNameToId[jointName] = jointId;
            continue;
        }

        if (inWorldbody && name == "freejoint" && bodyStack.size() > 1)
        {
            const dtDynIndex bodyId = bodyStack[bodyStack.size() - 1];
            const std::string jointName = AttributeOr(attrs, "name", "freejoint");
            const dtDynIndex jointId = spec.AddJoint(bodyId, jointName, dtDynJointType::Free);
            jointNameToId[jointName] = jointId;
            continue;
        }

        if (inWorldbody && name == "site" && bodyStack.size() > 1)
        {
            const dtDynIndex bodyId = bodyStack[bodyStack.size() - 1];
            const dtDynIndex siteId = spec.AddSite(bodyId, AttributeOr(attrs, "name", "site"));
            if (HasAttribute(attrs, "pos"))
            {
                spec.Sites()[siteId].pos = ParseVector3(attrs.at("pos"));
            }
            if (HasAttribute(attrs, "quat"))
            {
                spec.Sites()[siteId].quat = ParseVector4(attrs.at("quat"));
            }
            continue;
        }

        if (inActuator && name == "motor")
        {
            const std::string jointName = AttributeOr(attrs, "joint", "");
            const auto jointIt = jointNameToId.find(jointName);
            if (jointIt == jointNameToId.end())
            {
                throw std::runtime_error("dtDynSpec: actuator references unknown joint");
            }
            const dtDynIndex actuatorId =
                spec.AddActuator(jointIt->second, AttributeOr(attrs, "name", "motor"));
            spec.Actuators()[actuatorId].gear =
                RealAttributeOr(attrs, "gear", spec.Actuators()[actuatorId].gear);
            continue;
        }
    }

    return spec;
}

const dtArray<dtDynBodySpec> &dtDynSpec::Bodies() const { return m_bodies; }

const dtArray<dtDynJointSpec> &dtDynSpec::Joints() const { return m_joints; }

const dtArray<dtDynSiteSpec> &dtDynSpec::Sites() const { return m_sites; }

const dtArray<dtDynActuatorSpec> &dtDynSpec::Actuators() const { return m_actuators; }

dtArray<dtDynBodySpec> &dtDynSpec::Bodies() { return m_bodies; }

dtArray<dtDynJointSpec> &dtDynSpec::Joints() { return m_joints; }

dtArray<dtDynSiteSpec> &dtDynSpec::Sites() { return m_sites; }

dtArray<dtDynActuatorSpec> &dtDynSpec::Actuators() { return m_actuators; }

} // namespace dt::Dynamics
