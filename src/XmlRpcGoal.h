#include <XmlRpcValue.h>

class XmlRpcGoal : public XmlRpc::XmlRpcValue
{
  public:

    char getId(const char *name)
    {
      for (int i = 0; i < size(); i++)
      {
        XmlRpc::XmlRpcValue item = (*this)[i];
        std::string this_name = item["name"];
        if (this_name.compare(name) == 0)
        {
          return toChar(item["id"]);
        }
      }
      ROS_ERROR("XmlRpcGoal: no valid id found matching name '%s'", name);
      return 0;
    }

    std::string getName(char id)
    {
      for (int i = 0; i < size(); i++)
      {
        XmlRpc::XmlRpcValue item = (*this)[i];
        char this_id = toChar(item["id"]);
        if (this_id == id)
        {
          return item["name"];
        }
      }
      ROS_ERROR("XmlRpcGoal: no valid name found for id '%c'", id);
      return "";
    }

    bool checkPose(char id)
    {
      for (int i = 0; i < size(); i++)
      {
        XmlRpc::XmlRpcValue item = (*this)[i];
        char this_id = toChar(item["id"]);
        if (this_id == id)
        {
          XmlRpc::XmlRpcValue xmlpose = item["pose"];
          return (xmlpose.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                  xmlpose.size() == 7);
        }
      }
      return false;
    }

    geometry_msgs::Pose getPose(char id)
    {
      geometry_msgs::Pose pose;
      pose.position.x = pose.position.y = pose.position.z = 0.0;
      pose.orientation.x = pose.orientation.y = pose.orientation.z = pose.orientation.w = 0.0;
      for (int i = 0; i < size(); i++)
      {
        XmlRpc::XmlRpcValue item = (*this)[i];
        char this_id = toChar(item["id"]);
        if (this_id == id)
        {
          XmlRpc::XmlRpcValue xmlpose = item["pose"];
          ROS_ASSERT(xmlpose.getType() == XmlRpc::XmlRpcValue::TypeArray);
          ROS_ASSERT(xmlpose.size() == 7);
          pose.position.x = toNumber(xmlpose[0]);
          pose.position.y = toNumber(xmlpose[1]);
          pose.position.z = toNumber(xmlpose[2]);
          pose.orientation.x = toNumber(xmlpose[3]);
          pose.orientation.y = toNumber(xmlpose[4]);
          pose.orientation.z = toNumber(xmlpose[5]);
          pose.orientation.w = toNumber(xmlpose[6]);

          return pose;
        }
      }
      ROS_ERROR("XmlRpcGoal: no valid pose found for id '%c'", id);
      return pose;
    }

    double getVal(char id)
    {
      double val;
      for (int i = 0; i < size(); i++)
      {
        XmlRpc::XmlRpcValue item = (*this)[i];
        char this_id = toChar(item["id"]);
        if (this_id == id)
        {
          XmlRpc::XmlRpcValue xmlval = item["value"];
          return toNumber(xmlval);
        }
      }
      ROS_ERROR("XmlRpcGoal: no valid pose found for id '%c'", id);
      return 0.0;
    }

  private:

    double toNumber(XmlRpc::XmlRpcValue xmlVal)
    {
      double num = 0;
      if (xmlVal.getType() == XmlRpc::XmlRpcValue::TypeInt)
        num = static_cast<int>(xmlVal);
      else if (xmlVal.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        num = static_cast<double>(xmlVal);
      else
        ROS_ERROR("Incompatible type in array");
      return num;
    }

    char toChar(XmlRpc::XmlRpcValue xmlVal)
    {
      char ch = '?';
      if (xmlVal.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        std::string str = xmlVal;
        ch = *str.c_str();
      }
      else if (xmlVal.getType() == XmlRpc::XmlRpcValue::TypeInt)
        ch = '0' + static_cast<int>(xmlVal);
      else
        ROS_ERROR("Incompatible type in array");
      return ch;
    }
};

