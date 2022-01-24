#!/usr/bin/env python
import rospy
import os
from robomaker_simulation_msgs.srv import *
from robomaker_simulation_msgs.msg import Tag
from robomaker_client import RobomakerClient

JOB_ARN = os.environ.get('AWS_ROBOMAKER_SIMULATION_JOB_ARN')
ROBOMAKER_CLIENT = RobomakerClient(os.environ.get('AWS_REGION'))


def on_add_tags(request):
    #TODO: handle the case of duplicate tag keys
    if len(request.tags) == 0:
        return AddTagsResponse(success=False, message='Empty list of tags')
    tag_map = {tag.key: tag.value for (tag) in request.tags}
    status, msg = ROBOMAKER_CLIENT.add_tags(JOB_ARN, tag_map)
    return AddTagsResponse(success=status, message=msg)


def on_list_tags(request):
    status, res = ROBOMAKER_CLIENT.list_tags(JOB_ARN)
    if status is False:
        return ListTagsResponse(success=False, message=res)
    tags = [Tag(key=key, value=value) for key, value in res.iteritems()]
    return ListTagsResponse(success=True, tags=tags)


def on_remove_tags(request):
    if len(request.keys) == 0:
        return RemoveTagsResponse(success=False, message='Empty list of keys')
    status, msg = ROBOMAKER_CLIENT.remove_tags(JOB_ARN, request.keys)
    return RemoveTagsResponse(success=status, message=msg)


def on_cancel(request):
    status, msg = ROBOMAKER_CLIENT.cancel_simulation_job(JOB_ARN)
    return CancelResponse(success=status, message=msg)


def robomaker_server():
    if JOB_ARN is None:
        print 'Job ARN is set to {JOB_ARN} and is invalid. Exiting...'.format(JOB_ARN=JOB_ARN)
        return
    print('JOB ARN is set to {JOB_ARN}'.format(JOB_ARN=JOB_ARN))
    rospy.init_node('robomaker_simulation')
    rospy.Service('job/cancel', Cancel, on_cancel)
    rospy.Service('job/add_tags', AddTags, on_add_tags)
    rospy.Service('job/list_tags', ListTags, on_list_tags)
    rospy.Service('job/remove_tags', RemoveTags, on_remove_tags)
    print('RoboMaker simulation ros service ready')
    rospy.spin()
