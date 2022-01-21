"""Module for accessing RoboMaker service using boto3 API."""
import boto3
import botocore
import traceback

from .session import RefreshableSession

class RobomakerClient(object):

    def __init__(self, region):
        """
        Initialize the boto3 client.
        :param region: (str) Target region: eu-west-1, us-east-1, us-west-2, etc
        :return: boto3.session.client: boto3 client
        """
        if region is None:
            msg = "Region cannot be None."
            print(msg)
            raise ValueError(msg)
        session = RefreshableSession()
        self.client = session.client(
            service_name='robomaker',
            region_name=region,
        )

    def cancel_simulation_job(self, arn):
        """
        Cancel a simulation job.
        :param arn: (str) arn of the simulation job
        :return:
            bool: status of service request
            string: failure message
       """
        print('Cancelling simulation {arn}'.format(arn=arn))
        def caller(): return self.client.cancel_simulation_job(job=arn)
        status, msg, _ = _call_handler(caller)
        return status, msg

    def list_tags(self, arn):
        """
        List tags associated with an job
        :param arn: (str) arn of the simulation job
        :return:
            bool: status of service request
            string: failure message
            dict: map of tags associated with the job
        """
        def caller(): return self.client.list_tags_for_resource(resourceArn=arn)
        status, msg, res = _call_handler(caller)
        if status is True:
            return True, res['tags']
        return False, msg

    def add_tags(self, arn, tags):
        """
        Adds tags to a particular job
        :param arn: (str) arn of the simulation job
        :param tags: (dict) a map of tags
        :return:
            bool: status of service request
            string: failure message
        """
        def caller(): return self.client.tag_resource(resourceArn=arn, tags=tags)
        status, msg, _ = _call_handler(caller)
        return status, msg

    def remove_tags(self, arn, keys):
        """
        Removes tags from a job
        :param arn: (str) arn of the simulation job
        :param keys: (list) list of tag keys to be removed from job
        :return:
            bool: status of service request
            string: failure message
        """
        def caller(): return self.client.untag_resource(resourceArn=arn, tagKeys=keys)
        status, msg, _ = _call_handler(caller)
        return status, msg


def _call_handler(caller):
    """
    This function calls AWS RoboMaker service.
    :param caller: a function wrapper to invoke in order to call appropriate API
    :return:
        bool: status of service request
        string: failure message
        dict: raw response back from API
    """
    try:
        res = caller()
        response_https_code = res['ResponseMetadata']['HTTPStatusCode']
        if 200 <= response_https_code < 300:
            return True, '', res
    except botocore.exceptions.ClientError as e:
        traceback.print_exc()
        code = e.response['Error']['Code']
        message = e.response['Error']['Message']
        print('Failed to call RoboMaker service, client error: {response}'.format(response=e.response))
        return False, 'Failed to complete request, {code}: {message}'.format(code=code, message=message), None
    except Exception as e:
        traceback.print_exc()
        print('Failed to call RoboMaker service, error: {err}'.format(err=e.message))
    return False, 'Failed to complete request', None




