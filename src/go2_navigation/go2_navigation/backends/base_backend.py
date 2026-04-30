from abc import ABC, abstractmethod

from geometry_msgs.msg import PoseStamped


class NavigationBackend(ABC):
    @abstractmethod
    def send_goal(self, goal: PoseStamped) -> bool:
        raise NotImplementedError

    @abstractmethod
    def cancel(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def status(self) -> str:
        raise NotImplementedError
