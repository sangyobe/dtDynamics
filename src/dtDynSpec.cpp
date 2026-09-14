/*!
 \file      dtDynSpec.cpp
 \brief     Editable native dtDynamics model specification definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.1
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynSpec.h"

#include "dtDynamics/dtDynValidation.h"

#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace dt::Dynamics
{
namespace
{

template <typename Item>
dtDynIndex FindNamedObject(const dtArray<Item> &items, const std::string &name)
{
    for (dtDynIndex index = 0; index < static_cast<dtDynIndex>(items.size()); ++index)
    {
        if (items[static_cast<std::size_t>(index)].name == name)
        {
            return index;
        }
    }
    return DTDYN_INVALID_INDEX;
}

std::string FormatXmlErrorMessage(const std::string &sourcePath,
                                  const std::size_t offset,
                                  const std::string &message)
{
    const std::string source = sourcePath.empty() ? "dtDynSpec::DecodeXml" : "dtDynSpec::LoadXmlFile '" + sourcePath + "'";
    return source + " at offset " + std::to_string(offset) + ": " + message;
}

} // namespace

dtDynXmlError::dtDynXmlError(const dtDynXmlErrorCode code,
                             std::string sourcePath,
                             const std::size_t offset,
                             const std::string &message)
    : std::runtime_error(FormatXmlErrorMessage(sourcePath, offset, message)),
      m_code(code),
      m_sourcePath(std::move(sourcePath)),
      m_offset(offset)
{
}

dtDynXmlErrorCode dtDynXmlError::Code() const noexcept
{
    return m_code;
}

const std::string &dtDynXmlError::SourcePath() const noexcept
{
    return m_sourcePath;
}

std::size_t dtDynXmlError::Offset() const noexcept
{
    return m_offset;
}

namespace
{

struct AttributeValue
{
    std::string text;
    std::size_t offset = 0;
};

using AttributeMap = std::unordered_map<std::string, AttributeValue>;

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
    while (pos < tag.size() && !std::isspace(static_cast<unsigned char>(tag[pos])) && tag[pos] != '/')
    {
        ++pos;
    }
    return tag.substr(0, pos);
}

struct ElementFrame
{
    std::string name;
    std::size_t offset = 0;
};

bool IsSupportedElement(const std::string &name)
{
    return name == "mujoco" || name == "compiler" || name == "option" ||
           name == "worldbody" || name == "body" || name == "inertial" ||
           name == "joint" || name == "freejoint" || name == "site" ||
           name == "actuator" || name == "motor";
}

bool IsValidTagName(const std::string &name)
{
    if (name.empty() ||
        !(std::isalpha(static_cast<unsigned char>(name[0])) || name[0] == '_'))
    {
        return false;
    }
    for (const char character : name)
    {
        if (!(std::isalnum(static_cast<unsigned char>(character)) || character == '_' ||
              character == '-' || character == ':' || character == '.'))
        {
            return false;
        }
    }
    return true;
}

std::size_t FindFirstNonWhitespace(const std::string &xml,
                                   const std::size_t begin,
                                   const std::size_t end)
{
    for (std::size_t offset = begin; offset < end; ++offset)
    {
        if (!std::isspace(static_cast<unsigned char>(xml[offset])))
        {
            return offset;
        }
    }
    return std::string::npos;
}

std::size_t FindTagEnd(const std::string &xml,
                       const std::size_t begin,
                       const std::string &sourcePath)
{
    char quote = '\0';
    for (std::size_t offset = begin + 1; offset < xml.size(); ++offset)
    {
        const char character = xml[offset];
        if (quote != '\0')
        {
            if (character == quote)
            {
                quote = '\0';
            }
            continue;
        }
        if (character == '"' || character == '\'')
        {
            quote = character;
        }
        else if (character == '>')
        {
            return offset;
        }
    }
    throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                        "unterminated XML tag");
}

bool HasSupportedElementParent(const std::string &name, const std::string &parent)
{
    if (name == "compiler" || name == "option" || name == "worldbody" ||
        name == "actuator")
    {
        return parent == "mujoco";
    }
    if (name == "body")
    {
        return parent == "worldbody" || parent == "body";
    }
    if (name == "inertial" || name == "joint" || name == "freejoint" || name == "site")
    {
        return parent == "body";
    }
    if (name == "motor")
    {
        return parent == "actuator";
    }
    return true;
}

dtDynXmlErrorCode XmlCodeFromValidation(const dtDynValidationCode code)
{
    if (code == dtDynValidationCode::NonFiniteValue)
    {
        return dtDynXmlErrorCode::NonFiniteValue;
    }
    if (code == dtDynValidationCode::DuplicateName)
    {
        return dtDynXmlErrorCode::DuplicateName;
    }
    if (code == dtDynValidationCode::UnsupportedJoint)
    {
        return dtDynXmlErrorCode::UnsupportedElement;
    }
    return dtDynXmlErrorCode::InvalidReference;
}

std::size_t ValidationOrigin(const dtDynValidationError &error,
                             const dtArray<std::size_t> &bodyOrigins,
                             const dtArray<std::size_t> &jointOrigins,
                             const dtArray<std::size_t> &siteOrigins,
                             const dtArray<std::size_t> &actuatorOrigins)
{
    const dtArray<std::size_t> *origins = &bodyOrigins;
    if (error.object == dtDynObjectType::Joint)
    {
        origins = &jointOrigins;
    }
    else if (error.object == dtDynObjectType::Site)
    {
        origins = &siteOrigins;
    }
    else if (error.object == dtDynObjectType::Actuator)
    {
        origins = &actuatorOrigins;
    }
    if (error.index >= 0 && static_cast<std::size_t>(error.index) < origins->size())
    {
        return (*origins)[static_cast<std::size_t>(error.index)];
    }
    return 0;
}

AttributeMap ParseAttributes(const std::string &tag, const std::size_t tagOffset,
                             const std::string &sourcePath)
{
    AttributeMap attrs;
    std::size_t pos = TagName(tag).size();
    while (pos < tag.size())
    {
        while (pos < tag.size() && std::isspace(static_cast<unsigned char>(tag[pos])))
        {
            ++pos;
        }
        if (pos >= tag.size())
        {
            break;
        }
        if (tag[pos] == '/')
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                tagOffset + pos,
                                "self-closing marker must be the final non-whitespace token");
        }

        const std::size_t keyStart = pos;
        while (pos < tag.size() && tag[pos] != '=' && !std::isspace(static_cast<unsigned char>(tag[pos])))
        {
            ++pos;
        }
        const std::string key = tag.substr(keyStart, pos - keyStart);
        if (!IsValidTagName(key))
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                tagOffset + keyStart,
                                "invalid XML attribute name");
        }

        while (pos < tag.size() && std::isspace(static_cast<unsigned char>(tag[pos])))
        {
            ++pos;
        }
        if (key.empty() || pos >= tag.size() || tag[pos] != '=')
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                tagOffset + keyStart, "malformed XML attribute");
        }
        ++pos;
        while (pos < tag.size() && std::isspace(static_cast<unsigned char>(tag[pos])))
        {
            ++pos;
        }
        if (pos >= tag.size() || (tag[pos] != '"' && tag[pos] != '\''))
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                tagOffset + keyStart,
                                "XML attribute value must be quoted");
        }

        const char quote = tag[pos++];
        const std::size_t valueStart = pos;
        while (pos < tag.size() && tag[pos] != quote)
        {
            if (tag[pos] == '<')
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                    tagOffset + pos,
                                    "raw '<' is not allowed in an XML attribute value");
            }
            ++pos;
        }
        if (pos == tag.size())
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                tagOffset + keyStart,
                                "unterminated XML attribute value");
        }
        if (attrs.find(key) != attrs.end())
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                tagOffset + keyStart,
                                "duplicate XML attribute '" + key + "'");
        }
        attrs[key] = {tag.substr(valueStart, pos - valueStart), tagOffset + valueStart};
        ++pos;
    }
    return attrs;
}

bool HasAttribute(const AttributeMap &attrs, const std::string &key)
{
    return attrs.find(key) != attrs.end();
}

std::string AttributeOr(const AttributeMap &attrs, const std::string &key, const std::string &fallback)
{
    const auto it = attrs.find(key);
    return it == attrs.end() ? fallback : it->second.text;
}

dtDynReal ParseReal(const AttributeValue &attribute, const std::string &sourcePath)
{
    std::size_t first = 0;
    while (first < attribute.text.size() &&
           std::isspace(static_cast<unsigned char>(attribute.text[first])))
    {
        ++first;
    }
    std::size_t last = attribute.text.size();
    while (last > first &&
           std::isspace(static_cast<unsigned char>(attribute.text[last - 1])))
    {
        --last;
    }
    const std::size_t tokenOffset = first == attribute.text.size() ? attribute.offset : attribute.offset + first;
    const AttributeValue token = {attribute.text.substr(first, last - first), tokenOffset};
    std::size_t consumed = 0;
    try
    {
        const dtDynReal value = static_cast<dtDynReal>(std::stod(token.text, &consumed));
        if (consumed != token.text.size())
        {
            throw dtDynXmlError(dtDynXmlErrorCode::InvalidNumber, sourcePath,
                                token.offset + consumed,
                                "invalid numeric value '" + token.text + "'");
        }
        if (!std::isfinite(value))
        {
            throw dtDynXmlError(dtDynXmlErrorCode::NonFiniteValue, sourcePath, token.offset,
                                "numeric value must be finite: '" + token.text + "'");
        }
        return value;
    }
    catch (const dtDynXmlError &)
    {
        throw;
    }
    catch (const std::invalid_argument &)
    {
        throw dtDynXmlError(dtDynXmlErrorCode::InvalidNumber, sourcePath, token.offset,
                            "invalid numeric value '" + token.text + "'");
    }
    catch (const std::out_of_range &)
    {
        throw dtDynXmlError(dtDynXmlErrorCode::InvalidNumber, sourcePath, token.offset,
                            "numeric value is out of range: '" + token.text + "'");
    }
}

dtDynReal RealAttributeOr(const AttributeMap &attrs, const std::string &key,
                          const dtDynReal fallback, const std::string &sourcePath)
{
    const auto it = attrs.find(key);
    return it == attrs.end() ? fallback : ParseReal(it->second, sourcePath);
}

void RequireUniqueName(const std::string &name,
                       const AttributeMap &attrs,
                       std::unordered_set<std::string> &names,
                       const std::size_t tagOffset,
                       const std::string &objectType,
                       const std::string &sourcePath)
{
    if (!names.insert(name).second)
    {
        const auto attribute = attrs.find("name");
        const std::size_t offset = attribute == attrs.end() ? tagOffset : attribute->second.offset;
        throw dtDynXmlError(dtDynXmlErrorCode::DuplicateName, sourcePath, offset,
                            "duplicate " + objectType + " name '" + name + "'");
    }
}

dtArray<dtDynReal> ParseVectorValues(const AttributeValue &attribute, const std::size_t expectedCount,
                                     const std::string &sourcePath)
{
    dtArray<AttributeValue> tokens;
    std::size_t pos = 0;
    while (pos < attribute.text.size())
    {
        while (pos < attribute.text.size() &&
               std::isspace(static_cast<unsigned char>(attribute.text[pos])))
        {
            ++pos;
        }
        if (pos == attribute.text.size())
        {
            break;
        }
        const std::size_t tokenStart = pos;
        while (pos < attribute.text.size() &&
               !std::isspace(static_cast<unsigned char>(attribute.text[pos])))
        {
            ++pos;
        }
        tokens.push_back({attribute.text.substr(tokenStart, pos - tokenStart),
                          attribute.offset + tokenStart});
    }

    if (tokens.size() != expectedCount)
    {
        const std::size_t offset = tokens.size() > expectedCount ? tokens[expectedCount].offset : attribute.offset;
        throw dtDynXmlError(dtDynXmlErrorCode::InvalidVectorArity, sourcePath, offset,
                            "expected " + std::to_string(expectedCount) +
                                " vector values but found " + std::to_string(tokens.size()));
    }

    dtArray<dtDynReal> values;
    for (const AttributeValue &token : tokens)
    {
        values.push_back(ParseReal(token, sourcePath));
    }
    return values;
}

dt::Math::Vector3<dtDynReal> ParseVector3(const AttributeValue &attribute,
                                          const std::string &sourcePath)
{
    const dtArray<dtDynReal> values = ParseVectorValues(attribute, 3, sourcePath);
    dt::Math::Vector3<dtDynReal> value;
    value(0) = values[0];
    value(1) = values[1];
    value(2) = values[2];
    return value;
}

dt::Math::Vector4<dtDynReal> ParseVector4(const AttributeValue &attribute,
                                          const std::string &sourcePath)
{
    const dtArray<dtDynReal> values = ParseVectorValues(attribute, 4, sourcePath);
    dt::Math::Vector4<dtDynReal> value;
    value(0) = values[0];
    value(1) = values[1];
    value(2) = values[2];
    value(3) = values[3];
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
    return FormatReal(value(0)) + " " + FormatReal(value(1)) + " " +
           FormatReal(value(2)) + " " + FormatReal(value(3));
}

void SetIdentityQuat(dt::Math::Vector4<dtDynReal> &quat)
{
    quat(0) = 1.0;
    quat(1) = 0.0;
    quat(2) = 0.0;
    quat(3) = 0.0;
}

dtDynJointType JointTypeFromXml(const AttributeValue &attribute,
                                const std::string &sourcePath)
{
    if (attribute.text == "free")
    {
        return dtDynJointType::Free;
    }
    if (attribute.text == "ball")
    {
        return dtDynJointType::Ball;
    }
    if (attribute.text == "slide")
    {
        return dtDynJointType::Slide;
    }
    if (attribute.text == "hinge")
    {
        return dtDynJointType::Hinge;
    }
    throw dtDynXmlError(dtDynXmlErrorCode::UnsupportedElement, sourcePath, attribute.offset,
                        "unsupported joint type '" + attribute.text + "'");
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

dtDynReal AngleScaleFromCompilerAttribute(const AttributeMap &attrs,
                                          const std::string &sourcePath)
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

    const auto attribute = attrs.find("angle");
    const std::size_t offset = attribute == attrs.end() ? 0 : attribute->second.offset;
    throw dtDynXmlError(dtDynXmlErrorCode::UnsupportedElement, sourcePath, offset,
                        "unsupported compiler angle unit '" + angle + "'");
}

void EncodeBody(const dtDynSpec &spec,
                const dtDynIndex bodyId,
                const dtDynIndex depth,
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
        out << indent << "  <inertial pos=\"" << FormatVector3(body.inertial.pos)
            << "\" mass=\"" << FormatReal(body.inertial.mass)
            << "\" diaginertia=\"" << FormatVector3(body.inertial.inertia) << "\"";
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
        out << indent << "  <joint name=\"" << joint.name
            << "\" type=\"" << JointTypeToXml(joint.type)
            << "\" pos=\"" << FormatVector3(joint.pos)
            << "\" axis=\"" << FormatVector3(joint.axis)
            << "\" armature=\"" << FormatReal(joint.armature) << "\"";
        if (joint.limited)
        {
            out << " limited=\"true\" range=\"" << FormatReal(joint.rangeMin)
                << " " << FormatReal(joint.rangeMax) << "\"";
        }
        out << "/>\n";
    }

    for (const dtDynIndex siteId : body.sites)
    {
        const dtDynSiteSpec &site = spec.Sites()[static_cast<std::size_t>(siteId)];
        out << indent << "  <site name=\"" << site.name
            << "\" pos=\"" << FormatVector3(site.pos) << "\"";
        if (site.quat(0) != 0.0 || site.quat(1) != 0.0 || site.quat(2) != 0.0 || site.quat(3) != 0.0)
        {
            out << " quat=\"" << FormatVector4(site.quat) << "\"";
        }
        out << "/>\n";
    }

    for (dtDynIndex child = body.firstChild; child >= 0; child = spec.Bodies()[static_cast<std::size_t>(child)].nextSibling)
    {
        EncodeBody(spec, child, depth + 1, out);
    }

    out << indent << "</body>\n";
}

} // namespace

dtDynSpec::dtDynSpec()
{
    Clear();
}

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

dtDynIndex dtDynSpec::AddJoint(const dtDynIndex body, const std::string &name, const dtDynJointType type)
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
    for (dtDynIndex child = m_bodies[0].firstChild; child >= 0; child = m_bodies[static_cast<std::size_t>(child)].nextSibling)
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
                out << " joint=\"" << m_joints[static_cast<std::size_t>(actuator.joint)].name << "\"";
            }
            out << " gear=\"" << FormatReal(actuator.gear) << "\"/>\n";
        }
        out << "  </actuator>\n";
    }
    out << "</mujoco>\n";
    return out.str();
}

void dtDynSpec::SaveXmlFile(const std::string &path) const
{
    const std::string xml = EncodeXml();
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        throw std::runtime_error("dtDynSpec::SaveXmlFile: failed to open XML file for writing: " + path);
    }

    output.write(xml.data(), static_cast<std::streamsize>(xml.size()));
    output.close();
    if (!output)
    {
        throw std::runtime_error("dtDynSpec::SaveXmlFile: failed to write XML file: " + path);
    }
}

dtDynSpec DecodeXmlImpl(const std::string &xml, const std::string &sourcePath)
{
    dtDynSpec spec;
    dtDynReal angleScale = kDegreeToRadian;
    dtArray<dtDynIndex> bodyStack;
    bodyStack.push_back(0);
    dtArray<ElementFrame> elementStack;
    dtArray<std::size_t> bodyOrigins;
    bodyOrigins.push_back(0);
    dtArray<std::size_t> jointOrigins;
    dtArray<std::size_t> siteOrigins;
    dtArray<std::size_t> actuatorOrigins;
    std::unordered_map<std::string, dtDynIndex> jointNameToId;
    std::unordered_set<std::string> bodyNames = {"world"};
    std::unordered_set<std::string> jointNames;
    std::unordered_set<std::string> siteNames;
    std::unordered_set<std::string> actuatorNames;
    bool sawMujocoRoot = false;
    bool sawMarkup = false;
    bool sawCompiler = false;
    bool sawWorldbody = false;
    bool sawActuator = false;

    std::size_t cursor = 0;
    while (true)
    {
        const std::size_t begin = xml.find('<', cursor);
        const std::size_t textEnd = begin == std::string::npos ? xml.size() : begin;
        const std::size_t unexpectedText = FindFirstNonWhitespace(xml, cursor, textEnd);
        if (unexpectedText != std::string::npos)
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, unexpectedText,
                                "unexpected text outside an XML element");
        }
        if (begin == std::string::npos)
        {
            break;
        }
        if (xml.compare(begin, 4, "<!--") == 0)
        {
            const std::size_t commentEnd = xml.find("-->", begin + 4);
            if (commentEnd == std::string::npos)
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                    "malformed XML comment");
            }
            sawMarkup = true;
            cursor = commentEnd + 3;
            continue;
        }
        const std::size_t end = FindTagEnd(xml, begin, sourcePath);

        const std::string rawTag = xml.substr(begin + 1, end - begin - 1);
        std::size_t leadingWhitespace = 0;
        while (leadingWhitespace < rawTag.size() &&
               std::isspace(static_cast<unsigned char>(rawTag[leadingWhitespace])))
        {
            ++leadingWhitespace;
        }
        std::size_t tagOffset = begin + 1 + leadingWhitespace;
        std::string tag = Trim(rawTag);
        cursor = end + 1;
        if (tag.empty())
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                "empty XML tag");
        }
        if (tag[0] == '?')
        {
            if (tag.back() != '?' || begin != 0 || sawMarkup || sawMujocoRoot)
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                    "malformed or misplaced XML declaration");
            }
            const std::string declaration = Trim(tag.substr(1, tag.size() - 2));
            if (TagName(declaration) != "xml")
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                    "unsupported XML processing instruction");
            }
            const AttributeMap declarationAttrs =
                ParseAttributes(declaration, tagOffset + 1, sourcePath);
            const auto version = declarationAttrs.find("version");
            if (version == declarationAttrs.end() ||
                (version->second.text != "1.0" && version->second.text != "1.1"))
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                    "XML declaration requires version 1.0 or 1.1");
            }
            for (const auto &entry : declarationAttrs)
            {
                if (entry.first != "version" && entry.first != "encoding" &&
                    entry.first != "standalone")
                {
                    throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                        entry.second.offset,
                                        "unsupported XML declaration attribute");
                }
                if (entry.first == "standalone" && entry.second.text != "yes" &&
                    entry.second.text != "no")
                {
                    throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                        entry.second.offset,
                                        "invalid XML standalone declaration value");
                }
            }
            sawMarkup = true;
            continue;
        }
        sawMarkup = true;
        if (tag[0] == '!')
        {
            throw dtDynXmlError(dtDynXmlErrorCode::UnsupportedElement, sourcePath, begin,
                                "unsupported XML declaration");
        }

        bool closing = false;
        if (tag[0] == '/')
        {
            closing = true;
            const std::string closingTag = tag.substr(1);
            std::size_t closingWhitespace = 0;
            while (closingWhitespace < closingTag.size() &&
                   std::isspace(static_cast<unsigned char>(closingTag[closingWhitespace])))
            {
                ++closingWhitespace;
            }
            tagOffset += 1 + closingWhitespace;
            tag = Trim(closingTag);
        }

        bool selfClosing = false;
        if (!tag.empty() && tag.back() == '/')
        {
            selfClosing = true;
            tag = Trim(tag.substr(0, tag.size() - 1));
        }

        const std::string name = TagName(tag);
        if (!IsValidTagName(name))
        {
            throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, tagOffset,
                                "invalid XML element name");
        }
        if (!IsSupportedElement(name))
        {
            throw dtDynXmlError(dtDynXmlErrorCode::UnsupportedElement, sourcePath, begin,
                                "unsupported XML element <" + name + ">");
        }
        AttributeMap attrs;
        if (closing)
        {
            std::size_t suffix = name.size();
            while (suffix < tag.size() &&
                   std::isspace(static_cast<unsigned char>(tag[suffix])))
            {
                ++suffix;
            }
            if (suffix != tag.size())
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                    tagOffset + suffix,
                                    "closing XML tags cannot contain attributes or suffix text");
            }
        }
        else
        {
            attrs = ParseAttributes(tag, tagOffset, sourcePath);
        }

        if (!closing)
        {
            if (name == "mujoco")
            {
                if (sawMujocoRoot || !elementStack.empty())
                {
                    throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                        "XML must contain exactly one mujoco root element");
                }
                sawMujocoRoot = true;
                bodyOrigins[0] = begin;
            }
            else if (!sawMujocoRoot || elementStack.empty())
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                    "XML element lies outside the mujoco root");
            }
            else if (!HasSupportedElementParent(name, elementStack[elementStack.size() - 1].name))
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                    "supported element <" + name + "> is not valid inside <" +
                                        elementStack[elementStack.size() - 1].name + ">");
            }

            bool *sectionSeen = nullptr;
            if (name == "compiler")
            {
                sectionSeen = &sawCompiler;
            }
            else if (name == "worldbody")
            {
                sectionSeen = &sawWorldbody;
            }
            else if (name == "actuator")
            {
                sectionSeen = &sawActuator;
            }
            if (sectionSeen != nullptr)
            {
                if (*sectionSeen)
                {
                    throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                        "duplicate <" + name + "> section");
                }
                *sectionSeen = true;
            }
        }

        if (closing)
        {
            if (selfClosing || elementStack.empty() ||
                elementStack[elementStack.size() - 1].name != name)
            {
                throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                    "misnested closing element </" + name + ">");
            }
            elementStack.resize(elementStack.size() - 1);
            if (name == "body")
            {
                if (bodyStack.size() <= 1)
                {
                    throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, begin,
                                        "closing body has no matching body frame");
                }
                bodyStack.resize(bodyStack.size() - 1);
            }
            continue;
        }

        if (!selfClosing)
        {
            elementStack.push_back({name, begin});
        }

        if (name == "worldbody")
        {
            continue;
        }
        if (name == "compiler")
        {
            angleScale = AngleScaleFromCompilerAttribute(attrs, sourcePath);
            continue;
        }
        if (name == "actuator")
        {
            continue;
        }

        if (name == "body")
        {
            const dtDynIndex parent = bodyStack[bodyStack.size() - 1];
            const std::string bodyName = AttributeOr(attrs, "name", "body");
            RequireUniqueName(bodyName, attrs, bodyNames, begin, "body", sourcePath);
            const dtDynIndex bodyId = spec.AddBody(bodyName, parent);
            bodyOrigins.push_back(begin);
            if (HasAttribute(attrs, "pos"))
            {
                spec.Bodies()[bodyId].pos = ParseVector3(attrs.at("pos"), sourcePath);
            }
            if (HasAttribute(attrs, "quat"))
            {
                spec.Bodies()[bodyId].quat = ParseVector4(attrs.at("quat"), sourcePath);
            }
            if (!selfClosing)
            {
                bodyStack.push_back(bodyId);
            }
            continue;
        }

        if (name == "inertial" && bodyStack.size() > 1)
        {
            dtDynInertialSpec &inertial = spec.Bodies()[bodyStack[bodyStack.size() - 1]].inertial;
            inertial.mass = RealAttributeOr(attrs, "mass", inertial.mass, sourcePath);
            if (HasAttribute(attrs, "pos"))
            {
                inertial.pos = ParseVector3(attrs.at("pos"), sourcePath);
            }
            if (HasAttribute(attrs, "quat"))
            {
                inertial.quat = ParseVector4(attrs.at("quat"), sourcePath);
            }
            if (HasAttribute(attrs, "diaginertia"))
            {
                inertial.inertia = ParseVector3(attrs.at("diaginertia"), sourcePath);
            }
            continue;
        }

        if (name == "joint" && bodyStack.size() > 1)
        {
            const dtDynIndex bodyId = bodyStack[bodyStack.size() - 1];
            const std::string jointName = AttributeOr(attrs, "name", "joint");
            RequireUniqueName(jointName, attrs, jointNames, begin, "joint", sourcePath);
            const auto typeAttribute = attrs.find("type");
            const dtDynJointType jointType = typeAttribute == attrs.end() ? dtDynJointType::Hinge : JointTypeFromXml(typeAttribute->second, sourcePath);
            const dtDynIndex jointId = spec.AddJoint(bodyId, jointName, jointType);
            jointOrigins.push_back(begin);
            dtDynJointSpec &joint = spec.Joints()[jointId];
            if (HasAttribute(attrs, "pos"))
            {
                joint.pos = ParseVector3(attrs.at("pos"), sourcePath);
            }
            if (HasAttribute(attrs, "axis"))
            {
                joint.axis = ParseVector3(attrs.at("axis"), sourcePath);
            }
            joint.armature = RealAttributeOr(attrs, "armature", joint.armature, sourcePath);
            const bool hasLimitedAttribute = HasAttribute(attrs, "limited");
            if (hasLimitedAttribute)
            {
                const AttributeValue &limited = attrs.at("limited");
                if (limited.text == "true" || limited.text == "1")
                {
                    joint.limited = true;
                }
                else if (limited.text == "false" || limited.text == "0")
                {
                    joint.limited = false;
                }
                else
                {
                    throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath,
                                        limited.offset,
                                        "limited must be true, false, 1, or 0");
                }
            }
            if (HasAttribute(attrs, "range"))
            {
                const dtArray<dtDynReal> range = ParseVectorValues(attrs.at("range"), 2, sourcePath);
                joint.rangeMin = range[0] * angleScale;
                joint.rangeMax = range[1] * angleScale;
                if (!hasLimitedAttribute)
                {
                    joint.limited = true;
                }
            }
            jointNameToId[jointName] = jointId;
            continue;
        }

        if (name == "freejoint" && bodyStack.size() > 1)
        {
            const dtDynIndex bodyId = bodyStack[bodyStack.size() - 1];
            const std::string jointName = AttributeOr(attrs, "name", "freejoint");
            RequireUniqueName(jointName, attrs, jointNames, begin, "joint", sourcePath);
            const dtDynIndex jointId = spec.AddJoint(bodyId, jointName, dtDynJointType::Free);
            jointOrigins.push_back(begin);
            jointNameToId[jointName] = jointId;
            continue;
        }

        if (name == "site" && bodyStack.size() > 1)
        {
            const dtDynIndex bodyId = bodyStack[bodyStack.size() - 1];
            const std::string siteName = AttributeOr(attrs, "name", "site");
            RequireUniqueName(siteName, attrs, siteNames, begin, "site", sourcePath);
            const dtDynIndex siteId = spec.AddSite(bodyId, siteName);
            siteOrigins.push_back(begin);
            if (HasAttribute(attrs, "pos"))
            {
                spec.Sites()[siteId].pos = ParseVector3(attrs.at("pos"), sourcePath);
            }
            if (HasAttribute(attrs, "quat"))
            {
                spec.Sites()[siteId].quat = ParseVector4(attrs.at("quat"), sourcePath);
            }
            continue;
        }

        if (name == "motor")
        {
            const std::string jointName = AttributeOr(attrs, "joint", "");
            const auto jointIt = jointNameToId.find(jointName);
            if (jointIt == jointNameToId.end())
            {
                const auto jointAttribute = attrs.find("joint");
                const std::size_t offset = jointAttribute == attrs.end() ? begin : jointAttribute->second.offset;
                throw dtDynXmlError(dtDynXmlErrorCode::InvalidReference, sourcePath, offset,
                                    "actuator references unknown joint '" + jointName + "'");
            }
            const std::string actuatorName = AttributeOr(attrs, "name", "motor");
            RequireUniqueName(actuatorName, attrs, actuatorNames, begin, "actuator", sourcePath);
            const dtDynIndex actuatorId =
                spec.AddActuator(jointIt->second, actuatorName);
            actuatorOrigins.push_back(begin);
            spec.Actuators()[actuatorId].gear = RealAttributeOr(attrs, "gear", spec.Actuators()[actuatorId].gear, sourcePath);
            continue;
        }
    }

    if (!elementStack.empty())
    {
        const ElementFrame &unclosed = elementStack[elementStack.size() - 1];
        throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, unclosed.offset,
                            "unclosed XML element <" + unclosed.name + ">");
    }
    if (!sawMujocoRoot)
    {
        throw dtDynXmlError(dtDynXmlErrorCode::MalformedSyntax, sourcePath, 0,
                            "missing mujoco root element");
    }

    dtDynValidationError validationError;
    if (!ValidateSpec(spec, &validationError))
    {
        const std::size_t offset = ValidationOrigin(validationError, bodyOrigins, jointOrigins,
                                                    siteOrigins, actuatorOrigins);
        throw dtDynXmlError(XmlCodeFromValidation(validationError.code), sourcePath, offset,
                            validationError.message);
    }
    return spec;
}

dtDynSpec dtDynSpec::DecodeXml(const std::string &xml)
{
    return DecodeXmlImpl(xml, "");
}

dtDynSpec dtDynSpec::LoadXmlFile(const std::string &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        throw std::runtime_error("dtDynSpec::LoadXmlFile: failed to open XML file: " + path);
    }

    std::string xml;
    char buffer[4096];
    while (true)
    {
        input.read(buffer, static_cast<std::streamsize>(sizeof(buffer)));
        const std::streamsize count = input.gcount();
        if (count > 0)
        {
            xml.append(buffer, static_cast<std::size_t>(count));
        }
        if (input.eof())
        {
            break;
        }
        if (input.fail())
        {
            throw std::runtime_error("dtDynSpec::LoadXmlFile: failed to read XML file: " + path);
        }
    }

    input.clear();
    input.close();
    if (input.fail())
    {
        throw std::runtime_error("dtDynSpec::LoadXmlFile: failed to close XML file after reading: " + path);
    }
    return DecodeXmlImpl(xml, path);
}

dtDynIndex dtDynSpec::BodyId(const std::string &name) const
{
    return FindNamedObject(m_bodies, name);
}

dtDynIndex dtDynSpec::JointId(const std::string &name) const
{
    return FindNamedObject(m_joints, name);
}

dtDynIndex dtDynSpec::SiteId(const std::string &name) const
{
    return FindNamedObject(m_sites, name);
}

dtDynIndex dtDynSpec::ActuatorId(const std::string &name) const
{
    return FindNamedObject(m_actuators, name);
}

const dtArray<dtDynBodySpec> &dtDynSpec::Bodies() const
{
    return m_bodies;
}

const dtArray<dtDynJointSpec> &dtDynSpec::Joints() const
{
    return m_joints;
}

const dtArray<dtDynSiteSpec> &dtDynSpec::Sites() const
{
    return m_sites;
}

const dtArray<dtDynActuatorSpec> &dtDynSpec::Actuators() const
{
    return m_actuators;
}

dtArray<dtDynBodySpec> &dtDynSpec::Bodies()
{
    return m_bodies; // polyspace DEFECT:BREAKING_DATA_ENCAPSULATION [Justified:Medium] "Editable setup-time container; callers preserve referential integrity before compile or encode."
}

dtArray<dtDynJointSpec> &dtDynSpec::Joints()
{
    return m_joints; // polyspace DEFECT:BREAKING_DATA_ENCAPSULATION [Justified:Medium] "Editable setup-time container; callers preserve referential integrity before compile or encode."
}

dtArray<dtDynSiteSpec> &dtDynSpec::Sites()
{
    return m_sites; // polyspace DEFECT:BREAKING_DATA_ENCAPSULATION [Justified:Medium] "Editable setup-time container; callers preserve referential integrity before compile or encode."
}

dtArray<dtDynActuatorSpec> &dtDynSpec::Actuators()
{
    return m_actuators; // polyspace DEFECT:BREAKING_DATA_ENCAPSULATION [Justified:Medium] "Editable setup-time container; callers preserve referential integrity before compile or encode."
}

} // namespace dt::Dynamics
