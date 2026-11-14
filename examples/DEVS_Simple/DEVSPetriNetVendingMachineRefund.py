"""Interactive Coke vending machine with change refunds."""

from math import inf

from socs.devs.TESBase import PortValue

from .DEVSPetriNetVendingMachine import VendingMachine, main


class VendingMachineRefund(VendingMachine):
    def __init__(self):
        super().__init__()
        self.add_input_ports("?refund")
        self.add_output_ports("!refund")
        self.add_place("RefundRequested")
        self.add_transition("Refund")
        self.add_input_arc("Credit", "Refund", "coin")

    def tau(self):
        if self.tokens("Pending"):
            return 3
        return 0 if self.tokens("RefundRequested") else inf

    def delta_x(self, event):
        if self.tokens("RefundRequested"):
            return False
        if event.port == "?1USD":
            self.receive_tokens(event)
            return False
        if self.tokens("Pending"):
            return False
        if event.port == "?Coke" and next(self.enabled_bindings("StartSale"), None) is not None:
            self.fire("StartSale")
            return True
        if event.port == "?refund" and self.tokens("Credit"):
            self.set_tokens("RefundRequested", [True])
            return True
        return False

    def delta_y(self):
        if self.tokens("Pending"):
            self.fire("Deliver")
            return PortValue("!Coke")
        if self.tokens("RefundRequested"):
            amount = len(self.tokens("Credit"))
            for _ in range(amount):
                self.fire("Refund")
            self.set_tokens("RefundRequested", [])
            return PortValue("!refund", amount)
        return None




if __name__ == "__main__":
    main(VendingMachineRefund, __doc__)
