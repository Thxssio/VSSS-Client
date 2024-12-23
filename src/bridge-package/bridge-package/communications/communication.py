from abc import ABC, abstractmethod


class communication(ABC):
    @abstractmethod
    def send_command(self, command):
        pass