# AWS Robomaker ROS Service

This package contains ROS service definitions for service endpoints provided
inside of an AWS RoboMaker simulation.

# Tag API

You can tag, untag, and list tags in your simulation job from the ROS command-line or in your ROS application while it is running. See [AWS RoboMaker documentation](https://docs.aws.amazon.com/robomaker/latest/dg/simulation-job-tags.html). You must have an IAM role with the permissions below. Replace account# with your account number.

```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": [
                "robomaker:TagResource",
                "robomaker:UntagResource",
                "robomaker:ListTagsForResource"
            ],
            "Resource": [
                "arn:aws:robomaker:*:account#:simulation-job/*"
            ],
            "Effect": "Allow"
        }
    ]
}
```

## AddTags

### Python

```python
import rclpy
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import AddTags

def add_tags_to_my_sim():
    # If not already initialized:
    rclpy.init()
    add_node = rclpy.create_node('add_my_tags')
    add_tags(add_node, [Tag(key="name", value="my_test")])

def add_tags(node, tags):
    client = node.create_client(AddTags, '/robomaker/job/add_tags')
    client.wait_for_service(timeout_sec=30)

    request = AddTags.Request()
    request.tags = tags
    future_response = client.call_async(request)
    rclpy.spin_until_future_complete(node, future_response)
    response = future_response.result()
    if not response.success:
        # AddTags failed
        print(response.message)
```

### C++

```

```

## RemoveTags

### Python

```python
import rclpy
from robomaker_simulation_msgs.srv import RemoveTags

def remove_tags_from_my_sim():
    # If not already initialized:
    rclpy.init()
    remove_node = rclpy.create_node('remove_my_tags')
    remove_tags(remove_node, ['name'])

def remove_tags(node, keys):
    client = node.create_client(RemoveTags, '/robomaker/job/remove_tags')
    client.wait_for_service(timeout_sec=30)

    request = RemoveTags.Request()
    request.keys = keys
    future_response = client.call_async(request)
    rclpy.spin_until_future_complete(node, future_response)
    response = future_response.result()
    if not response.success:
        # RemoveTags failed
        print(response.message)

```

### C++

```

```

## ListTags

### Python

```python
import rclpy
from robomaker_simulation_msgs.srv import ListTags

def list_tags_on_my_sim():
    # If not already initialized:
    rclpy.init()
    list_node = rclpy.create_node('list_my_tags')
    list_tags(list_node)

def list_tags(node):
    client = node.create_client(ListTags, '/robomaker/job/list_tags')
    client.wait_for_service(timeout_sec=30)
    future_response = client.call_async(ListTags.Request())
    rclpy.spin_until_future_complete(node, future_response)
    response = future_response.result()
    if response.success:
        # ListTags succeeded
        print(response.tags)
    else:
        # ListTags failed
        print(response.message)
```

### C++

```

```

## CancelSimulation API

You can cancel your simulation job from the ROS command-line or in your ROS application while it is running. See [AWS RoboMaker documentation](https://docs.aws.amazon.com/robomaker/latest/dg/simulation-job-playback-rosbags.html#simulation-job-playback-rosbags-cancel). You must have an IAM role with the permissions below. Replace account# with your account number. 

```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": [
                "robomaker:CancelSimulationJob"
            ],
            "Resource": [
                "arn:aws:robomaker:*:account#:simulation-job/*"
            ],
            "Effect": "Allow"
        }
    ]
}
```

### Python

```python
import rclpy
from robomaker_simulation_msgs.srv import Cancel

def cancel_job():
    # If not already initialized
    rclpy.init()
    node = rclpy.create_node('cancel_my_job')

    client = node.create_client(Cancel, '/robomaker/job/cancel')
    client.wait_for_service(timeout_sec=30)

    future_response = client.call_async(Cancel.Request())
    rclpy.spin_until_future_complete(node, future_response)
    response = future_response.result()
    if not response.success:
        # Cancel failed
        print(response.message)

```

## License

This library is licensed under the Apache 2.0 License. 
