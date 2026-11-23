"""Emit the initial RMFS orders as one event at simulation time zero."""

from math import inf

from socs.devs import DEVS
from socs.devs.TESBase import PortValue


class OrderGenerator(DEVS):
    def __init__(self, order_count=100):
        super().__init__("OrderGenerator")
        if not isinstance(order_count, int) or isinstance(order_count, bool) or order_count < 0:
            raise ValueError("order_count must be a nonnegative integer")
        self.order_count = order_count
        self.add_output_ports("!orders")

    def set_initial_state(self):
        self.sent = False

    def tau(self):
        return 0 if self.order_count and not self.sent else inf

    def delta_x(self, event):
        return False

    def delta_y(self):
        self.sent = True
        return PortValue("!orders", tuple(range(1, self.order_count + 1)))

    def get_state(self):
        return self.sent

    def set_state(self, state):
        self.sent = state
