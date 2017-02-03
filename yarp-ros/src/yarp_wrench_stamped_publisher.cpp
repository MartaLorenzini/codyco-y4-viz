#include <iostream>
#include <yarp/os/all.h>
#include <limits.h>
#include <cmath>

using namespace yarp::os;
using namespace std;

#include "String.h"
#include "std_msgs_Header.h"
#include "geometry_msgs_WrenchStamped.h"
#include "TickTime.h"

struct wrench {
    double Fx;
    double Fy;
    double Fz;
    double Mx;
    double My;
    double Mz;
  };

inline TickTime normalizeSecNSec(double yarpTimeStamp)
{
    uint64_t time = (uint64_t) (yarpTimeStamp * 1000000000UL);
    uint64_t nsec_part = (time % 1000000000UL);
    uint64_t sec_part = (time / 1000000000UL);
    TickTime ret;

    if (sec_part > UINT_MAX)
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    ret.sec  = sec_part;
    ret.nsec = nsec_part;
    return ret;
}

int main(int argc, char **argv){

    Network yarp;

    Node node("/yarp/yarp_wrench_stamped_publisher");

    yarp::os::Publisher<geometry_msgs_WrenchStamped> publisher;
    if (!publisher.topic("/icub/wrench_stamped")) {
        cerr<< "Failed to create publisher to /wrench_stamped\n";
        return -1;
      }

    // message declarations
    geometry_msgs_WrenchStamped wrenchMsg;
    std::string frame_id ("l_foot");
    wrenchMsg.header.frame_id= frame_id;

    double null_F = 0, null_M = 0;
    wrench msgStream;
    msgStream.Fx = 0.5;
    msgStream.Fy = 0.5;
    msgStream.Fz = 1;
    msgStream.Mx = 0.5;
    msgStream.My = 0.5;
    msgStream.Mz = -1.1;

    //msg.stream = getdata();

    while (true){

        //compose WrenchStamped Msg
        wrenchMsg.header.stamp = normalizeSecNSec(yarp::os::Time::now());
        wrenchMsg.wrench.force.x = msgStream.Fx;
        wrenchMsg.wrench.force.y = msgStream.Fy;
        wrenchMsg.wrench.force.z = msgStream.Fz;
        wrenchMsg.wrench.torque.x = msgStream.Mx;
        wrenchMsg.wrench.torque.y = msgStream.My;
        wrenchMsg.wrench.torque.z = msgStream.Mz;

        // Send the wrench
        publisher.write(wrenchMsg);

        // wait some time to avoid flooding with messages
        Time::delay(0.1);
      }
    return 0;
  }
