# Robomaker ROS Service

This package contains ROS service definitions for service endpoints provided
inside of an AWS RoboMaker simulation.

# Usage

## SetTag

### Python

```

```

### C++

```

```

## RemoveTag

### Python

```
import rospy
from robomaker_simulation_msgs.srv import *

request = CancelRequest()
cancel_func = rospy.ServiceProxy('robomaker/job/cancel', )
resp = cancel_func(request)
if not resp.success:
	# Cancellation failed
	print resp.message

```

### C++

```

```

## ListTags

### Python

```
import rospy
from robomaker_simulation_msgs.srv import *



def cancel():
	request = CancelRequest()
	cancel_func = rospy.ServiceProxy('robomaker/job/cancel', )
	cancel_func(request)

```

### C++

```

```

## CancelSimulation

```python
import rospy
from robomaker_simulation_msgs.srv import *



def cancel():
	request = CancelRequest()
	cancel_func = rospy.ServiceProxy('robomaker/job/cancel', )
	cancel_func(request)
```