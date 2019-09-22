#include <XmlRpcValue.h>

class XmlRpcGoal : public XmlRpc::XmlRpcValue
{
  public:
    std::string getName(int id)
    {
      for (int i = 0; i < size(); i++)
      {
        XmlRpc::XmlRpcValue goal = (*this)[i];
        int this_id = goal["id"];
        if (this_id == id)
        {
          return goal["name"];
        }
      }
      ROS_ERROR("XmlRpcGoal: no valid name found for id %d", id);
      return "";
    }

    geometry_msgs::Pose getPose(int id)
    {
      geometry_msgs::Pose pose;
      for (int i = 0; i < size(); i++)
      {
        XmlRpc::XmlRpcValue goal = (*this)[i];
        int this_id = goal["id"];
        if (this_id == id)
        {
          XmlRpc::XmlRpcValue xmlpose = goal["pose"];
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
      ROS_ERROR("XmlRpcGoal: no valid pose found for id %d", id);
      return pose;
    }

  private:
    double toNumber(XmlRpc::XmlRpcValue xmlVal)
    {
      double num = 0;
      if(xmlVal.getType() == XmlRpc::XmlRpcValue::TypeInt)
        num = static_cast<int>(xmlVal);
      else if(xmlVal.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        num = static_cast<double>(xmlVal);
      else
        ROS_ERROR("Incompatible type in array");
      return num;
    }

};

