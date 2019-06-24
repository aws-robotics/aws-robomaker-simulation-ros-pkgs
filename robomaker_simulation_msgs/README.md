# AWS RoboMaker ROS Message Service

This package contains ROS service definitions for service endpoints provided inside of an AWS RoboMaker simulation.

## Usage

### AddTags

#### Python

```python
import rospy
from robomaker_simulation_msgs.srv import AddTags

def add_tags_to_my_sim():
    add_tags([Tag(key="name", value="my_test")])

def add_tags(tags):
    timeoutInSeconds = 60
    rospy.wait_for_service('/robomaker/job/add_tags', timeout=timeoutInSeconds)
    requestAddTags = rospy.ServiceProxy('/robomaker/job/add_tags', AddTags)
    response = requestAddTags(tags)
    if not response.success:
        # AddTags failed
        print response.message

```

### RemoveTag

#### Python

```python
import rospy
from robomaker_simulation_msgs.srv import RemoveTags

def remove_tags_from_my_sim():
    remove_tags([Tag(key="name", value="my_test")])

def remove_tags(tags):
    timeoutInSeconds = 60
    rospy.wait_for_service('/robomaker/job/remove_tags', timeout=timeoutInSeconds)
    requestRemoveTags = rospy.ServiceProxy('/robomaker/job/remove_tags', RemoveTags)
    response = requestRemoveTags(tags)
    if not response.success:
        # RemoveTags failed
        print response.message

```

### ListTags

#### Python

```python
import rospy
from robomaker_simulation_msgs.srv import ListTags

def list_tags():
    timeoutInSeconds = 60
    rospy.wait_for_service('/robomaker/job/list_tags', timeout=timeoutInSeconds)
    requestListTags = rospy.ServiceProxy('/robomaker/job/list_tags', ListTags)
    response = requestListTags()
    if response.success:
        # ListTags succeeded
        print response.tags
    else:
        # ListTags failed
        print response.message

```

### CancelSimulation
#### Python

```python
import rospy
from robomaker_simulation_msgs.srv import Cancel

def cancel_job():
    timeoutInSeconds = 60
    rospy.wait_for_service('/robomaker/job/cancel', timeout=timeoutInSeconds)
    requestCancel = rospy.ServiceProxy('/robomaker/job/cancel', Cancel)
    response = requestCancel()
    if not response.success:
        # Request cancel job failed
        print response.message
```

## License

This library is licensed under the Apache 2.0 License. 
