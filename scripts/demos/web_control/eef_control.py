"""FK/IK services and atomically planned flange chunks."""


class EefControl:
    def get_arms(self):
        with self.lock:
            return {name: chain.metadata() for name, chain in self.kinematics.items()}

    def _chain(self, arm):
        if not isinstance(arm, str) or arm not in self.kinematics:
            raise ValueError("Unknown arm or kinematics unsupported for this robot")
        return self.kinematics[arm]

    def fk(self, arm, joint_positions=None):
        with self.lock:
            if self.phase != "ready":
                raise RuntimeError("Simulator is not ready")
            chain = self._chain(arm)
            q = (
                joint_positions
                if joint_positions is not None
                else [self.positions[self.names.index(n)] for n in chain.joint_names]
            )
        return chain.fk(q)

    def ik(self, arm, position, quaternion, seed=None):
        with self.lock:
            if self.phase != "ready":
                raise RuntimeError("Simulator is not ready")
            chain = self._chain(arm)
            q = seed if seed is not None else [self.positions[self.names.index(n)] for n in chain.joint_names]
        if seed is None:
            q = [min(hi, max(lo, value)) for value, (lo, hi) in zip(q, chain.limits)]
        return chain.ik(position, quaternion, q)

    def execute_eef_chunk(self, arm, positions, quaternions, steps_per_action):
        with self.lock:
            chain = self._chain(arm)
            if self.phase != "ready" or self.active_task or self.planning or self.actions:
                raise RuntimeError("Robot busy or not ready")
            if (
                not isinstance(positions, list)
                or not isinstance(quaternions, list)
                or not 1 <= len(positions) <= 128
                or len(positions) != len(quaternions)
            ):
                raise ValueError("Expected matching positions/quaternions with 1 to 128 rows")
            if type(steps_per_action) is not int or not 1 <= steps_per_action <= 120000:
                raise ValueError("Invalid steps_per_action")
            # Validate the entire request before doing potentially expensive IK.
            from kinematics import transform

            for p, q in zip(positions, quaternions):
                transform(p, q)
            seed = [
                min(hi, max(lo, self.positions[self.names.index(n)]))
                for n, (lo, hi) in zip(chain.joint_names, chain.limits)
            ]
            self.planning = True
        try:
            rows = []
            for index, (p, q) in enumerate(zip(positions, quaternions)):
                try:
                    seed = chain.ik(p, q, seed)["joint_positions"]
                except ValueError as exc:
                    raise ValueError(f"EEF row {index}: {exc}") from exc
                rows.append(seed)
            with self.lock:
                self.planning = False
                task = self.execute_joint_chunk(chain.joint_names, rows, steps_per_action)
                self.tasks[task["id"]].update(arm=arm, control="eef", solved_joint_positions=rows)
                self.record_event(
                    {
                        "type": "eef_chunk",
                        "task_id": task["id"],
                        "arm": arm,
                        "positions": positions,
                        "quaternions": quaternions,
                    }
                )
                return self.get_task(task["id"])
        finally:
            with self.lock:
                self.planning = False
