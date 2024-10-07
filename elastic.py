"""
This is ElasticLib V2025.0.0 Beta 1 ported into python. Source can be found at https://github.com/Gold872/elastic-dashboard/tree/v2025.0.0-beta-1
"""

from enum import Enum
from ntcore import NetworkTableInstance, PubSubOptions

class NotificationLevel(Enum):
    INFO = 0,
    WARNING = 1,
    ERROR = 2

class ElasticNotification:
    def __init__(self, level: NotificationLevel, title: str, description: str) -> None:
        self.level = level
        self.title = title
        self.description = description

class Elastic:

    _topic = NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications")
    _publisher = _topic.publish(PubSubOptions(sendAll=True, keepDuplicates=True))

    @staticmethod
    def send_alert(alert: ElasticNotification) -> None:
        Elastic._publisher.set("{" + f"\"level\":\"{alert.level.name}\",\"title\": \"{alert.title}\",\"description\":\"{alert.description}\"" + "}")
