# AWS RoboMaker ROS Service Proxy

ROS service to proxy AWS RoboMaker Cancel and Tag API calls from within a ROS application.

### Setup

Make sure you have `boto3` installed in the environment you run this package. `boto3` via the `package.xml` is very old
and can cause issues so we recommend installation of `boto3` via `pip`.

### Usage:

Launchfile:
```
roslaunch robomaker_simulation_service start.launch
```

As an executable:
```
./robomaker_server
```

### Covered APIs:

Cancel: https://docs.aws.amazon.com/robomaker/latest/dg/API_CancelSimulationJob.html
ListTags: https://docs.aws.amazon.com/robomaker/latest/dg/API_ListTagsForResource.html
AddTags: https://docs.aws.amazon.com/robomaker/latest/dg/API_TagResource.html
RemoveTags: https://docs.aws.amazon.com/robomaker/latest/dg/API_UntagResource.html

### Additional Info:

http://wiki.ros.org/rospy/Overview/Services
http://wiki.ros.org/srv

