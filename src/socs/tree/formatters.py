"""Formatting helpers for DecisionTree path output."""

from math import isinf, isnan
from typing import Any

from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot


class SnapshotFormatter:
    """Format a SystemSnapshot with engine and recursive model state details.

    The formatter is callable with the ``(node_id, snapshot)`` signature used
    by :meth:`DecisionTree.format_path`.
    """

    def __init__(
        self,
        indent: int = 2,
        simulation_time_bound: float | None = None,
        show_state_percentage: bool = False,
    ) -> None:
        if not isinstance(indent, int) or isinstance(indent, bool) or indent < 0:
            raise ValueError("indent must be a non-negative integer.")
        if not isinstance(show_state_percentage, bool):
            raise TypeError("show_state_percentage must be a bool.")
        self.indent = indent
        self.simulation_time_bound = simulation_time_bound
        self.show_state_percentage = show_state_percentage

    def __call__(self, node_id: int, snapshot: SystemSnapshot) -> str:
        if not isinstance(snapshot, SystemSnapshot):
            raise TypeError("snapshot must be a SystemSnapshot instance.")

        lines = [f"Node {node_id}", "  Snapshot:"]
        prefix = " " * (2 + self.indent)
        lines.extend(
            [
                f"{prefix}Simulation time: {self._format_value(snapshot.t_now)}",
                "{0}Next continuous event: {1}".format(
                    prefix, self._format_value(snapshot.t_next_continuous)
                ),
                "{0}Next animation event: {1}".format(
                    prefix, self._format_value(snapshot.t_next_animation)
                ),
                f"{prefix}Simulation time bound: "
                f"{self._format_value(self.simulation_time_bound) if self.simulation_time_bound is not None else '<not provided>'}",
                f"{prefix}Pending external inputs: {len(snapshot.x_buffer)}",
            ]
        )

        model_paths = self._model_paths(snapshot.root_model)
        for index, (target_id, event) in enumerate(snapshot.x_buffer, start=1):
            target = model_paths.get(target_id, repr(target_id))
            lines.append(
                f"{prefix}  XBuffer[{index}]: target={target}, event={event}"
            )

        self._append_model(lines, snapshot.root_model, prefix, depth=0)
        return "\n".join(lines)

    def _append_model(
        self,
        lines: list[str],
        model: ModelSnapshot,
        prefix: str,
        depth: int,
    ) -> None:
        model_prefix = prefix + "  " * depth
        lines.append(f"{model_prefix}Model: {model.model_path}")

        common_state_keys = ["time_last", "time_next"]
        if self.show_state_percentage:
            common_state_keys.append("state_percentage")

        for key in common_state_keys:
            if key in model.common_state:
                lines.append(
                    f"{model_prefix}  {key}: "
                    f"{self._format_value(model.common_state[key])}"
                )

        if model.has_model_state:
            lines.append(
                f"{model_prefix}  model_state: {self._format_value(model.model_state)}"
            )

        for child in model.children.values():
            self._append_model(lines, child, prefix, depth + 1)

    @classmethod
    def _model_paths(cls, root_model: ModelSnapshot) -> dict[int, str]:
        paths = {}

        def visit(model: ModelSnapshot) -> None:
            paths[model.model_id] = model.model_path
            for child in model.children.values():
                visit(child)

        visit(root_model)
        return paths

    @staticmethod
    def _format_value(value: Any) -> str:
        if isinstance(value, float):
            if isnan(value):
                return "nan"
            if isinf(value):
                return "inf" if value > 0 else "-inf"
            return f"{value:.6f}".rstrip("0").rstrip(".")
        return repr(value)
