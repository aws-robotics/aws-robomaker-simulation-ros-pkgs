from unittest.mock import MagicMock, patch
import unittest

from aws_robomaker_cloudwatch_publisher import CloudWatchPublisher



class TestCloudWatchPublisher(unittest.TestCase):
    METRIC_NAME = "TestMetricName"
    METRIC_VALUE = 42

    @unittest.mock.patch("aws_robomaker_cloudwatch_publisher.cloudwatch_publisher.boto3")
    def setUp(self, mock_obj):
        self.boto3_mock = mock_obj
        self.client_mock = MagicMock()
        self.boto3_mock.client.return_value = self.client_mock
        self.publisher = CloudWatchPublisher(TestCloudWatchPublisher.METRIC_NAME)

    def test_client_constructed_on_init(self):
        self.boto3_mock.client.assert_called_once()

    def test_metric_name_stored_correctly(self):
        self.assertEqual(self.publisher.metric_name, TestCloudWatchPublisher.METRIC_NAME)

    def test_metric_publishes_correctly(self):
        self.publisher.put_metric(TestCloudWatchPublisher.METRIC_VALUE)
        obj = self.client_mock.put_metric_data.call_args_list[0][1]
        self.assertEqual(obj["Namespace"], "AWS/RoboMaker")
        metric_data = obj["MetricData"][0]

        self.assertEqual(metric_data["MetricName"], TestCloudWatchPublisher.METRIC_NAME)
        self.assertEqual(metric_data["Value"], TestCloudWatchPublisher.METRIC_VALUE)
        self.assertIsNotNone(metric_data["Timestamp"])

    @patch("aws_robomaker_cloudwatch_publisher.cloudwatch_publisher.os")
    def test_metric_job_id_correct(self, os_mock):
        os_mock.environ.get.return_value = "TestJobId"
        self.publisher.put_metric(TestCloudWatchPublisher.METRIC_VALUE)
        obj = self.client_mock.put_metric_data.call_args_list[0][1]
        job_id = obj["MetricData"][0]["Dimensions"][0]["Value"]
        self.assertEqual(job_id, "TestJobId")

