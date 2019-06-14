# AWS RoboMaker ROS Message Service

This package contains ROS service definitions for service endpoints provided inside of an AWS RoboMaker simulation.

# Usage

## AddTags

### Python

```
import rospy
from robomaker_simulation_msgs.srv import AddTags

def add_tags(tags):
    requestAddTags = rospy.ServiceProxy('/robomaker/job/add_tags', AddTags)
    response = requestAddTags(tags)
    if not response.success:
        # AddTags failed
        print response.message

```

## RemoveTag

### Python

```
import rospy
from robomaker_simulation_msgs.srv import RemoveTags

def remove_tags(tags):
    requestRemoveTags = rospy.ServiceProxy('/robomaker/job/remove_tags', RemoveTags)
    response = requestRemoveTags(tags)
    if not response.success:
        # RemoveTags failed
        print response.message

```

## ListTags

### Python

```
import rospy
from robomaker_simulation_msgs.srv import ListTags

def list_tags():
    requestListTags = rospy.ServiceProxy('/robomaker/job/list_tags', ListTags)
    response = requestListTags()
    if response.success:
        # ListTags succeeded
        print response.tags
    else:
        # ListTags failed
        print response.message

```

## CancelSimulation
### Python

```
import rospy
from robomaker_simulation_msgs.srv import Cancel

def cancel_job():
    requestCancel = rospy.ServiceProxy('/robomaker/job/cancel', Cancel)
    response = requestCancel()
    if not response.success:
        # Request cancel job failed
        print response.message
```