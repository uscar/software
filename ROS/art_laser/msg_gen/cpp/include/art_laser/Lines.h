/* Auto-generated by genmsg_cpp for file /home/art/ROS/art_laser/msg/Lines.msg */
#ifndef ART_LASER_MESSAGE_LINES_H
#define ART_LASER_MESSAGE_LINES_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Polygon.h"

namespace art_laser
{
template <class ContainerAllocator>
struct Lines_ {
  typedef Lines_<ContainerAllocator> Type;

  Lines_()
  : theta_index()
  , est_rho()
  , est_theta()
  , delta_rho()
  , endpoints()
  , lengths()
  , endpoint_ranges()
  , theta()
  {
  }

  Lines_(const ContainerAllocator& _alloc)
  : theta_index(_alloc)
  , est_rho(_alloc)
  , est_theta(_alloc)
  , delta_rho(_alloc)
  , endpoints(_alloc)
  , lengths(_alloc)
  , endpoint_ranges(_alloc)
  , theta(_alloc)
  {
  }

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _theta_index_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  theta_index;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _est_rho_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  est_rho;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _est_theta_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  est_theta;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _delta_rho_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  delta_rho;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _endpoints_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  endpoints;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _lengths_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  lengths;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _endpoint_ranges_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  endpoint_ranges;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _theta_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  theta;


  ROS_DEPRECATED uint32_t get_theta_index_size() const { return (uint32_t)theta_index.size(); }
  ROS_DEPRECATED void set_theta_index_size(uint32_t size) { theta_index.resize((size_t)size); }
  ROS_DEPRECATED void get_theta_index_vec(std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other > & vec) const { vec = this->theta_index; }
  ROS_DEPRECATED void set_theta_index_vec(const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other > & vec) { this->theta_index = vec; }
  ROS_DEPRECATED uint32_t get_est_rho_size() const { return (uint32_t)est_rho.size(); }
  ROS_DEPRECATED void set_est_rho_size(uint32_t size) { est_rho.resize((size_t)size); }
  ROS_DEPRECATED void get_est_rho_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->est_rho; }
  ROS_DEPRECATED void set_est_rho_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->est_rho = vec; }
  ROS_DEPRECATED uint32_t get_est_theta_size() const { return (uint32_t)est_theta.size(); }
  ROS_DEPRECATED void set_est_theta_size(uint32_t size) { est_theta.resize((size_t)size); }
  ROS_DEPRECATED void get_est_theta_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->est_theta; }
  ROS_DEPRECATED void set_est_theta_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->est_theta = vec; }
  ROS_DEPRECATED uint32_t get_delta_rho_size() const { return (uint32_t)delta_rho.size(); }
  ROS_DEPRECATED void set_delta_rho_size(uint32_t size) { delta_rho.resize((size_t)size); }
  ROS_DEPRECATED void get_delta_rho_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->delta_rho; }
  ROS_DEPRECATED void set_delta_rho_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->delta_rho = vec; }
  ROS_DEPRECATED uint32_t get_endpoints_size() const { return (uint32_t)endpoints.size(); }
  ROS_DEPRECATED void set_endpoints_size(uint32_t size) { endpoints.resize((size_t)size); }
  ROS_DEPRECATED void get_endpoints_vec(std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) const { vec = this->endpoints; }
  ROS_DEPRECATED void set_endpoints_vec(const std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) { this->endpoints = vec; }
  ROS_DEPRECATED uint32_t get_lengths_size() const { return (uint32_t)lengths.size(); }
  ROS_DEPRECATED void set_lengths_size(uint32_t size) { lengths.resize((size_t)size); }
  ROS_DEPRECATED void get_lengths_vec(std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) const { vec = this->lengths; }
  ROS_DEPRECATED void set_lengths_vec(const std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) { this->lengths = vec; }
  ROS_DEPRECATED uint32_t get_endpoint_ranges_size() const { return (uint32_t)endpoint_ranges.size(); }
  ROS_DEPRECATED void set_endpoint_ranges_size(uint32_t size) { endpoint_ranges.resize((size_t)size); }
  ROS_DEPRECATED void get_endpoint_ranges_vec(std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) const { vec = this->endpoint_ranges; }
  ROS_DEPRECATED void set_endpoint_ranges_vec(const std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) { this->endpoint_ranges = vec; }
  ROS_DEPRECATED uint32_t get_theta_size() const { return (uint32_t)theta.size(); }
  ROS_DEPRECATED void set_theta_size(uint32_t size) { theta.resize((size_t)size); }
  ROS_DEPRECATED void get_theta_vec(std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) const { vec = this->theta; }
  ROS_DEPRECATED void set_theta_vec(const std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) { this->theta = vec; }
private:
  static const char* __s_getDataType_() { return "art_laser/Lines"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "5e886f1edefc993f520f05d33f4e8b92"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32[] theta_index\n\
float32[] est_rho\n\
float32[] est_theta\n\
float32[] delta_rho\n\
geometry_msgs/Polygon[] endpoints\n\
geometry_msgs/Polygon[] lengths\n\
geometry_msgs/Polygon[] endpoint_ranges\n\
geometry_msgs/Polygon[] theta\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
geometry_msgs/Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, theta_index);
    ros::serialization::serialize(stream, est_rho);
    ros::serialization::serialize(stream, est_theta);
    ros::serialization::serialize(stream, delta_rho);
    ros::serialization::serialize(stream, endpoints);
    ros::serialization::serialize(stream, lengths);
    ros::serialization::serialize(stream, endpoint_ranges);
    ros::serialization::serialize(stream, theta);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, theta_index);
    ros::serialization::deserialize(stream, est_rho);
    ros::serialization::deserialize(stream, est_theta);
    ros::serialization::deserialize(stream, delta_rho);
    ros::serialization::deserialize(stream, endpoints);
    ros::serialization::deserialize(stream, lengths);
    ros::serialization::deserialize(stream, endpoint_ranges);
    ros::serialization::deserialize(stream, theta);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(theta_index);
    size += ros::serialization::serializationLength(est_rho);
    size += ros::serialization::serializationLength(est_theta);
    size += ros::serialization::serializationLength(delta_rho);
    size += ros::serialization::serializationLength(endpoints);
    size += ros::serialization::serializationLength(lengths);
    size += ros::serialization::serializationLength(endpoint_ranges);
    size += ros::serialization::serializationLength(theta);
    return size;
  }

  typedef boost::shared_ptr< ::art_laser::Lines_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::art_laser::Lines_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Lines
typedef  ::art_laser::Lines_<std::allocator<void> > Lines;

typedef boost::shared_ptr< ::art_laser::Lines> LinesPtr;
typedef boost::shared_ptr< ::art_laser::Lines const> LinesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::art_laser::Lines_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::art_laser::Lines_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace art_laser

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::art_laser::Lines_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::art_laser::Lines_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::art_laser::Lines_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5e886f1edefc993f520f05d33f4e8b92";
  }

  static const char* value(const  ::art_laser::Lines_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5e886f1edefc993fULL;
  static const uint64_t static_value2 = 0x520f05d33f4e8b92ULL;
};

template<class ContainerAllocator>
struct DataType< ::art_laser::Lines_<ContainerAllocator> > {
  static const char* value() 
  {
    return "art_laser/Lines";
  }

  static const char* value(const  ::art_laser::Lines_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::art_laser::Lines_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32[] theta_index\n\
float32[] est_rho\n\
float32[] est_theta\n\
float32[] delta_rho\n\
geometry_msgs/Polygon[] endpoints\n\
geometry_msgs/Polygon[] lengths\n\
geometry_msgs/Polygon[] endpoint_ranges\n\
geometry_msgs/Polygon[] theta\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
geometry_msgs/Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const  ::art_laser::Lines_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::art_laser::Lines_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.theta_index);
    stream.next(m.est_rho);
    stream.next(m.est_theta);
    stream.next(m.delta_rho);
    stream.next(m.endpoints);
    stream.next(m.lengths);
    stream.next(m.endpoint_ranges);
    stream.next(m.theta);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Lines_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::art_laser::Lines_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::art_laser::Lines_<ContainerAllocator> & v) 
  {
    s << indent << "theta_index[]" << std::endl;
    for (size_t i = 0; i < v.theta_index.size(); ++i)
    {
      s << indent << "  theta_index[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.theta_index[i]);
    }
    s << indent << "est_rho[]" << std::endl;
    for (size_t i = 0; i < v.est_rho.size(); ++i)
    {
      s << indent << "  est_rho[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.est_rho[i]);
    }
    s << indent << "est_theta[]" << std::endl;
    for (size_t i = 0; i < v.est_theta.size(); ++i)
    {
      s << indent << "  est_theta[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.est_theta[i]);
    }
    s << indent << "delta_rho[]" << std::endl;
    for (size_t i = 0; i < v.delta_rho.size(); ++i)
    {
      s << indent << "  delta_rho[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.delta_rho[i]);
    }
    s << indent << "endpoints[]" << std::endl;
    for (size_t i = 0; i < v.endpoints.size(); ++i)
    {
      s << indent << "  endpoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.endpoints[i]);
    }
    s << indent << "lengths[]" << std::endl;
    for (size_t i = 0; i < v.lengths.size(); ++i)
    {
      s << indent << "  lengths[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.lengths[i]);
    }
    s << indent << "endpoint_ranges[]" << std::endl;
    for (size_t i = 0; i < v.endpoint_ranges.size(); ++i)
    {
      s << indent << "  endpoint_ranges[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.endpoint_ranges[i]);
    }
    s << indent << "theta[]" << std::endl;
    for (size_t i = 0; i < v.theta.size(); ++i)
    {
      s << indent << "  theta[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.theta[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // ART_LASER_MESSAGE_LINES_H

