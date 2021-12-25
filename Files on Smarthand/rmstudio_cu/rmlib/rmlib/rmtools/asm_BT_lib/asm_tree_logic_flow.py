import py_trees

##### Behaviors ##### ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class Toggle_Var(py_trees.behaviour.Behaviour):  # BASIC

    N = 0

    def __init__(self, keyString="toggleVar_", initVal=1, ctrl=None):
        """Create a key and populate it with an initial value"""
        super().__init__(name="Toggle_Var")
        self._DEBUG = 0
        self.ctrl = ctrl  # Should be an RMStudio object, will raise an error if not set
        self.__class__.N += 1
        self.key = keyString + str(self.__class__.N)
        self.val = initVal
        self.initVal = initVal
        ASMBB.set(self.key, self.val)
        if self._DEBUG:
            print("Created `Toggle_Var` with key:", self.key)

    def update(self):
        """Toggle the value"""
        self.val = ASMBB.get(self.key) * -1.0


class Force_Query_DECO(py_trees.decorators.Decorator):
    """ """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.force_trace = []

    def update(self):
        robot = self.decorated.ctrl
        wrench = robot.arm.get_unbiased_force()
        self.force_trace.append(wrench)
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


# === Negation Decorator ===


class Negator_DECO(py_trees.decorators.Decorator):
    """
    Reverse SUCCESS and FAILURE, otherwise return underlying status
    """

    def update(self):
        """
        Return the decorated child's status unless it is
        :data:`~py_trees.common.Status.SUCCESS` in which case, return
        :data:`~py_trees.common.Status.FAILURE`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Negate:" + (
                " [%s]" % self.decorated.feedback_message
                if self.decorated.feedback_message
                else ""
            )
            return py_trees.common.Status.FAILURE
        elif self.decorated.status == py_trees.common.Status.FAILURE:
            self.feedback_message = "Negate:" + (
                " [%s]" % self.decorated.feedback_message
                if self.decorated.feedback_message
                else ""
            )
            return py_trees.common.Status.SUCCESS
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


# ___ End Negation ___


# ==== Run to X Failures Decorator ====


class Run_to_X_Failures_DECO(py_trees.decorators.Decorator):
    """
    Dont stop running, until a certain number of failures have been seen.  Reset count on success
    """

    def __init__(
        self,
        child: py_trees.behaviour.Behaviour,
        X_allowedFails=5,
        memory=0,
        name="Run_to_X_Failures_DECO",
    ):
        """Create a decorator that r"""
        super().__init__(child=child, name=name)
        self.limit = X_allowedFails
        self.reset()
        self.memory = memory
        self.stopped = False

    def reset(self):
        """Reset deco to an un-run state"""
        self.count = 1

    def update(self):
        """
        Return the decorated child's status unless it is
        :data:`~py_trees.common.Status.FAILURE` in which case, return
        :data:`~py_trees.common.Status.RUNNING`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if not self.stopped:
            # 0. If failed, then print message, and increment failure count
            if self.decorated.status == py_trees.common.Status.FAILURE:
                self.feedback_message = (
                    "Allowed "
                    + str(self.count)
                    + " of "
                    + str(self.limit)
                    + (
                        " [%s]" % self.decorated.feedback_message
                        if self.decorated.feedback_message
                        else ""
                    )
                )
                self.count += 1
                # 1. If the limit has not been exceeded, return Running
                if self.count <= self.limit:
                    return py_trees.common.Status.RUNNING
                # 2. Else the limit was exceeded, reset count, and return Failure
                else:
                    if self.memory:
                        self.stopped = True
                    else:
                        self.reset()
                    return py_trees.common.Status.FAILURE
            # 3. Reset count if the successor succeeded
            if self.decorated.status == py_trees.common.Status.SUCCESS:
                #                 print( "Child node SUCCESS!" )
                self.reset()
            # TODO: RESET ON RUNNING?
            # 4. Else the child node is running or has succeeded, return that status
            self.feedback_message = self.decorated.feedback_message
            return self.decorated.status
        else:
            return py_trees.common.Status.FAILURE


# ___ End Run to X Failures ___


class Always_Succeed(py_trees.behaviour.Behaviour):
    """Return SUCCESS if the gripped mass meets or exceeds expectations, otherwise return FAILURE"""

    # NOTE: This function assumes the robot is near enough to sea level for 'g' to apply

    def __init__(self):
        """Store the target mass"""
        super().__init__(name="Always_Succeed")

    def update(self):
        """
        Every time is ticked, return status, also - Triggering, checking, monitoring, set feedback message. Anything...but do not block!:
        Check that the arm is moving, goal reached, or failed
        """
        self.logger.debug("  %s [Always_Succeed::update()]" % self.name)
        return py_trees.common.Status.SUCCESS
