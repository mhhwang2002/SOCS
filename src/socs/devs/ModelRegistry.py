
from uuid import UUID, uuid4


class ModelRegistry:
    """Maintain UUID-based lookup for a hierarchical model structure."""

    def __init__(self):
        self._models: dict[UUID, object] = {}

    def register(self, model):
        """Register a model and all its descendants."""
        entries = self._collect_models(model)

        # Phase 1: Validate all identities without modifying the registry.
        seen_ids = {}

        for current_model, model_id in entries:
            previous_model = seen_ids.get(model_id)

            if previous_model is not None and previous_model is not current_model:
                raise ValueError(
                    f"Model UUID {model_id} is assigned "
                    "to multiple models."
                )

            seen_ids[model_id] = current_model

            registered = self._models.get(model_id)

            if registered is not None and registered is not current_model:
                raise ValueError(
                    f"Model UUID {model_id} is already assigned "
                    "to another model."
                )

        # Phase 2: Assign UUIDs after all validations succeed.
        for current_model, model_id in entries:
            if getattr(current_model, "_model_id", None) is None:
                current_model._model_id = model_id

        # Phase 3: Update the registry.
        for current_model, model_id in entries:
            self._models[model_id] = current_model
            
                
    def unregister(self, model_or_id, recursive=True):
        """Unregister a model, optionally including its descendants."""
        if isinstance(model_or_id, UUID):
            model_id = model_or_id
            self._models.pop(model_id, None)
            return

        model = model_or_id

        if recursive:
            entries = self._collect_models(model)
            models_to_remove = [current for current, _ in entries]
        else:
            models_to_remove = [model]

        for current_model in models_to_remove:
            model_id = getattr(current_model, "_model_id", None)

            if model_id is not None:
                if self._models.get(model_id) is current_model:
                    del self._models[model_id]

    def get(self, model_id):
        """Return the model registered under the given UUID."""
        if not isinstance(model_id, UUID):
            raise TypeError("model_id must be a uuid.UUID.")

        try:
            return self._models[model_id]
        except KeyError:
            raise KeyError(f"Model UUID {model_id} is not registered.") from None

    def rebuild(self, root_model):
        """Replace the registry contents using a model hierarchy."""
        new_registry = ModelRegistry()
        new_registry.register(root_model)
        self._models = new_registry._models

    def __len__(self):
        return len(self._models)

    def __contains__(self, model_id):
        return model_id in self._models

    @staticmethod
    def _collect_models(root_model):
        """Collect a model hierarchy and validate its UUID assignments."""
        entries = []
        visited = set()

        def visit(model):
            object_id = id(model)

            if object_id in visited:
                raise ValueError(
                    "The model hierarchy contains a cycle "
                    "or a model referenced more than once."
                )

            visited.add(object_id)

            model_id = getattr(model, "_model_id", None)

            if model_id is None:
                model_id = uuid4()
            elif not isinstance(model_id, UUID):
                raise TypeError(
                    "A model's _model_id must be a uuid.UUID or None."
                )

            entries.append((model, model_id))

            children = getattr(model, "children", ())

            if children is None:
                children = ()

            try:
                child_iterator = iter(children)
            except TypeError:
                raise TypeError(
                    "A model's children attribute must be iterable."
                ) from None

            for child in child_iterator:
                visit(child)

        visit(root_model)
        return entries
