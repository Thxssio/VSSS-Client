from abc import ABC, abstractmethod
from copy import deepcopy


class command(ABC):
    @abstractmethod
    def generate_command(self, *args):
        pass

    def clone_command(self):
        return deepcopy(self.command)