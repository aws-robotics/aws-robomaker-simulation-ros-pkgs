import boto3
from datetime import datetime
import os


class CloudWatchPublisher(object):

    def __init__(self, metric_name):
        region = os.environ["AWS_REGION"]
        self._client = boto3.client("cloudwatch", region_name=region)
        self._metric_name = metric_name

    @property
    def metric_name(self):
        return self._metric_name

    def put_metric(self, val):
        """Put metric with given value into CloudWatch.

        Automatically populates the dimensions with the current job is, and
        the timestamp with the current time.

        This function does not check for success or failure of putting metrics.
        """
        job_id = os.environ.get("AWS_ROBOMAKER_SIMULATION_JOB_ID", "UNKNOWN")
        self._client.put_metric_data(
            Namespace="AWS/RoboMaker",
            MetricData=[
                {
                    "MetricName": self._metric_name,
                    "Dimensions": [
                        {
                            "Name": "SimulationJobId",
                            "Value": job_id
                        }
                    ],
                    "Timestamp": datetime.now(),
                    "Value": val,
                    "Unit": "None"
                }
            ]
        )
