import time


class StartsAndStops:
    name = None
    start_time = None
    stop_time = None
    path_id = None

    def __init__(self, name):
        self.name = name

    def set_path_id(self, path_id):
        self.path_id = path_id

    def mark_start(self, callback=None):
        if self.start_time is not None:
            raise Exception(f"mark_start called more than once on {self}")
        self.start_time = time.time()
        self._report_running_status("started", callback)

    def mark_stop(self, callback=None):
        if self.start_time is None:
            raise Exception(f"mark_stop called before mark_start on {self}")
        if self.stop_time is not None:
            raise Exception(f"mark_stop called more than once on {self}")
        self.stop_time = time.time()
        self._report_running_status("finished", callback)

    def _report_running_status(self, status, callback):
        if callback is None:
            return
        callback(
            self.path_id,
            {
                "type": "running-status-change",
                "running-status": status,
            },
        )

    @property
    def started(self):
        return self.start_time is not None

    @property
    def finished(self):
        return self.stop_time is not None

    @property
    def running(self):
        return self.started and not self.finished

    @property
    def running_status(self):
        if not self.started:
            return "Not started"
        if self.running:
            return "Running"
        return "Finished"

    @property
    def duration(self):
        if not self.started or not self.finished:
            return None

        return self.stop_time - self.start_time

    @property
    def running_time(self):
        if not self.started:
            return None

        return time.time() - self.start_time


class Step(StartsAndStops):
    def __init__(self, name, function, skip):
        super().__init__(name)
        self.function = function
        self.skip = skip
        if skip:
            self.status = "Skipped"
        else:
            self.status = "Not executed"

    def execute(self, status_change_callback):
        if self.skip:
            return

        self.status = "In progress"
        self.mark_start(status_change_callback)
        try:
            self.function()
        except Exception as e:
            self.status = f"Caught exception {e}"
            self.mark_stop(status_change_callback)
            raise
        else:
            self.status = "Completed"
            self.mark_stop(status_change_callback)

    def __str__(self):
        return f"<Step {self.name}, {self.status}>"

    def __getstate__(self):
        state = self.__dict__

        if "function" in state:
            state = state.copy()
            # Unpickleable
            del state["function"]

        return state


class Stage(StartsAndStops):
    name = None
    status = "Not executed"
    path_id = None
    is_a_stage = True
    children = None
    has_substages = False
    failed = False

    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_step(self, name, function, skip=False):
        self.children.append(Step(name, function, skip))

    def add_substage(self, stage):
        self.children.append(stage)
        self.has_substages = True

    def execute(self, status_change_callback):
        self.status = "Running"
        self.mark_start(status_change_callback)

        try:
            for child in self.children:
                child.execute(status_change_callback)
        except Exception as e:
            self.failed = True
            self.status = f"Caught exception {e}"
            self.mark_stop(status_change_callback)
            raise
        else:
            self.status = "Completed"
            self.mark_stop(status_change_callback)

    def __str__(self):
        return f"Stage {self.name} with children: {list(map(str, self.children))}"

    def set_path_id(self, path_id):
        super().set_path_id(path_id)
        for i, child in enumerate(self.children):
            child.set_path_id(path_id + [i])

    def get_by_path_id(self, path_id):
        if path_id == []:
            return self

        childno = path_id[0]
        child = self.children[childno]

        if isinstance(child, Stage):
            return child.get_by_path_id(path_id[1:])

        return child


class ExecutionPlan(StartsAndStops):
    name = "Scap"
    stages = None
    finalized = False
    path_id = []

    def __init__(self, name):
        super().__init__(name)
        self.stages = []

    def add_stage(self, stagename):
        stage = Stage(stagename)
        self.stages.append(stage)
        return stage

    def finalize(self):
        if self.finalized:
            return

        for i, stage in enumerate(self.stages):
            stage.set_path_id([i])

        self.finalized = True

    def execute(self, status_change_callback=None):
        self.finalize()

        self.mark_start(status_change_callback)
        try:
            for stageid, stage in enumerate(self.stages):
                stage.execute(status_change_callback)
        finally:
            self.mark_stop(status_change_callback)

    def get_by_path_id(self, path_id: list):
        assert isinstance(path_id, list)

        if path_id == []:
            return self

        stagenum = path_id[0]
        stage = self.stages[stagenum]
        return stage.get_by_path_id(path_id[1:])
