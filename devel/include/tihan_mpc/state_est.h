// Generated by gencpp from file tihan_mpc/state_est.msg
// DO NOT EDIT!


#ifndef TIHAN_MPC_MESSAGE_STATE_EST_H
#define TIHAN_MPC_MESSAGE_STATE_EST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace tihan_mpc
{
template <class ContainerAllocator>
struct state_est_
{
  typedef state_est_<ContainerAllocator> Type;

  state_est_()
    : header()
    , lat(0.0)
    , lon(0.0)
    , x(0.0)
    , y(0.0)
    , psi(0.0)
    , v(0.0)
    , v_long(0.0)
    , v_lat(0.0)
    , yaw_rate(0.0)
    , a_long(0.0)
    , a_lat(0.0)
    , df(0.0)  {
    }
  state_est_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , lat(0.0)
    , lon(0.0)
    , x(0.0)
    , y(0.0)
    , psi(0.0)
    , v(0.0)
    , v_long(0.0)
    , v_lat(0.0)
    , yaw_rate(0.0)
    , a_long(0.0)
    , a_lat(0.0)
    , df(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _lat_type;
  _lat_type lat;

   typedef double _lon_type;
  _lon_type lon;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _psi_type;
  _psi_type psi;

   typedef double _v_type;
  _v_type v;

   typedef double _v_long_type;
  _v_long_type v_long;

   typedef double _v_lat_type;
  _v_lat_type v_lat;

   typedef double _yaw_rate_type;
  _yaw_rate_type yaw_rate;

   typedef double _a_long_type;
  _a_long_type a_long;

   typedef double _a_lat_type;
  _a_lat_type a_lat;

   typedef double _df_type;
  _df_type df;





  typedef boost::shared_ptr< ::tihan_mpc::state_est_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tihan_mpc::state_est_<ContainerAllocator> const> ConstPtr;

}; // struct state_est_

typedef ::tihan_mpc::state_est_<std::allocator<void> > state_est;

typedef boost::shared_ptr< ::tihan_mpc::state_est > state_estPtr;
typedef boost::shared_ptr< ::tihan_mpc::state_est const> state_estConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tihan_mpc::state_est_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tihan_mpc::state_est_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tihan_mpc::state_est_<ContainerAllocator1> & lhs, const ::tihan_mpc::state_est_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.lat == rhs.lat &&
    lhs.lon == rhs.lon &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.psi == rhs.psi &&
    lhs.v == rhs.v &&
    lhs.v_long == rhs.v_long &&
    lhs.v_lat == rhs.v_lat &&
    lhs.yaw_rate == rhs.yaw_rate &&
    lhs.a_long == rhs.a_long &&
    lhs.a_lat == rhs.a_lat &&
    lhs.df == rhs.df;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tihan_mpc::state_est_<ContainerAllocator1> & lhs, const ::tihan_mpc::state_est_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tihan_mpc

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tihan_mpc::state_est_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tihan_mpc::state_est_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tihan_mpc::state_est_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tihan_mpc::state_est_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tihan_mpc::state_est_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tihan_mpc::state_est_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tihan_mpc::state_est_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c920bd35ee9bfa5fb5330660c621c0a";
  }

  static const char* value(const ::tihan_mpc::state_est_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9c920bd35ee9bfa5ULL;
  static const uint64_t static_value2 = 0xfb5330660c621c0aULL;
};

template<class ContainerAllocator>
struct DataType< ::tihan_mpc::state_est_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tihan_mpc/state_est";
  }

  static const char* value(const ::tihan_mpc::state_est_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tihan_mpc::state_est_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"float64 lat      # latitude (deg)\n"
"float64 lon      # longitude (deg)\n"
"\n"
"float64 x        # x coordinate (m)\n"
"float64 y        # y coordinate (m)\n"
"float64 psi      # yaw angle (rad)\n"
"float64 v        # speed (m/s)\n"
"\n"
"float64 v_long   # longitidunal velocity (m/s)\n"
"float64 v_lat    # lateral velocity (m/s)\n"
"float64 yaw_rate # w_z, yaw rate (rad/s)\n"
"\n"
"float64 a_long   # longitudinal acceleration (m/s^2)\n"
"float64 a_lat    # lateral acceleration (m/s^2)\n"
"float64 df       # front steering angle (rad)\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::tihan_mpc::state_est_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tihan_mpc::state_est_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.lat);
      stream.next(m.lon);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.psi);
      stream.next(m.v);
      stream.next(m.v_long);
      stream.next(m.v_lat);
      stream.next(m.yaw_rate);
      stream.next(m.a_long);
      stream.next(m.a_lat);
      stream.next(m.df);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct state_est_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tihan_mpc::state_est_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tihan_mpc::state_est_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "lat: ";
    Printer<double>::stream(s, indent + "  ", v.lat);
    s << indent << "lon: ";
    Printer<double>::stream(s, indent + "  ", v.lon);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "psi: ";
    Printer<double>::stream(s, indent + "  ", v.psi);
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "v_long: ";
    Printer<double>::stream(s, indent + "  ", v.v_long);
    s << indent << "v_lat: ";
    Printer<double>::stream(s, indent + "  ", v.v_lat);
    s << indent << "yaw_rate: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_rate);
    s << indent << "a_long: ";
    Printer<double>::stream(s, indent + "  ", v.a_long);
    s << indent << "a_lat: ";
    Printer<double>::stream(s, indent + "  ", v.a_lat);
    s << indent << "df: ";
    Printer<double>::stream(s, indent + "  ", v.df);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TIHAN_MPC_MESSAGE_STATE_EST_H
