
"""
Example of a coupled FD-DEVS model for a two-slot toaster.

The model contains two toaster components with independent heating times.
External input coupling routes push events to each toaster, and external
output coupling exposes each toaster's completion event.
"""

from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import SimEngine

from FDDEVSToaster import Toaster


def make_input_event(model):
    """Read and construct an input event for the coupled model."""
    while True:
        port_name = input(
            "Input port name ('?push1' or '?push2'; blank to cancel): "
        ).strip()

        if not port_name:
            return None

        if port_name in model.IP:
            return model, PortValue(model.IP[port_name])

        print(f"Invalid input port: {port_name}")


def makeModel(name="system", tt1=20, tt2=40):
    """Create the coupled two-slot toaster model."""
    toaster1 = Toaster("t1", tt1)
    toaster2 = Toaster("t2", tt2)

    model = CoupledDEVS(name, None)
    model.addComponents(toaster1, toaster2)

    model.addInputPorts("?push1", "?push2")
    model.addOutputPorts("!pop1", "!pop2")

    # External Input Coupling (EIC).
    model.addCouple(
        model.IP["?push1"],
        toaster1.IP["?push"],
    )
    model.addCouple(
        model.IP["?push2"],
        toaster2.IP["?push"],
    )

    # External Output Coupling (EOC).
    model.addCouple(
        toaster1.OP["!pop"],
        model.OP["!pop1"],
    )
    model.addCouple(
        toaster2.OP["!pop"],
        model.OP["!pop2"],
    )

    return model


def runSimulation(model):
    """Run the coupled model interactively."""
    engine = SimEngine(model, make_input_event)

    # Enable step-by-step discrete-event simulation.
    engine.StepByStep = True
    engine.sfactor = float("inf")

    engine.runInteractiveMenu()


def runVerification(model):
    """Report that verification is not implemented in this example."""
    print(
        "Verification is not available: "
        "the legacy VeriEngine integration has not been ported."
    )


def main():
    """Run the example's command-line menu."""
    print(
        'Coupled model "system" contains two independent toaster models.'
    )
    print("Use '?push1' or '?push2' to start the corresponding toaster.")
    print("Heating times: t1 = 20, t2 = 40.")

    model = makeModel("system", 20, 40)

    while True:
        command = input("[s]imulate, [v]erify, e[x]it: ").strip().lower()

        if command in ("s", "sim", "simulate"):
            runSimulation(model)
        elif command in ("v", "veri", "verify"):
            runVerification(model)
        elif command in ("x", "exit"):
            break
        else:
            print(f"Invalid command: {command}")

    print("Goodbye!")


if __name__ == "__main__":
    main()
