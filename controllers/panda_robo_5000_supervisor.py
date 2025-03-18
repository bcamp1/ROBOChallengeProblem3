from controller import Supervisor
import roboticstoolbox as rtb
import spatialmath as sm
import math
import numpy as np

class PandaProtoSupervisor(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # 1) Create the Panda robot model (for IK) from roboticstoolbox
        self.rtb_panda = rtb.models.Panda()  # or rtb.models.DH.Panda()

        # Optional: If your Panda base in Webots is offset, set base transform.
        # Example: shift the Panda up 0.2 m in Z and 0.0 m in X/Y:
        # self.rtb_panda.base = sm.SE3(0.0, 0.0, 0.2)

        # 2) Retrieve the top-level PROTO node with DEF "panda"
        self.panda_node = self.getFromDef("panda")
        if self.panda_node is None:
            print("Error: Could not find a PROTO or node with DEF=panda in the .wbt file.")
            return

        # 3) Store references to each joint sub-PROTO by name.
        #    We assume your .wbt / .proto defines these 7 sub-protos:
        #      DEF panda_joint1
        #      DEF panda_joint2
        #      ...
        #      DEF panda_joint7
        #    Each must implement setJointPosition(angle_in_radians).
        self.joint_proto_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]
        self.joint_nodes = []

        for name in self.joint_proto_names:
            joint_node = self.panda_node.getFromProtoDef(name)
            if joint_node is None:
                print(f"Warning: Could not find sub-PROTO with DEF={name}")
            self.joint_nodes.append(joint_node)

    def run(self):
        path = self.generate_cu_path()

        # Use a gentler orientation so it's more likely within reach
        # e.g., pitch = -π/4 => pen angled 45° from vertical
        desired_rpy = (0, -math.pi / 4, 0)

        # Provide a "ready" posture for IK's initial guess
        # (tweak these if your robot starts in a different pose)
        q_ready = np.array([0.0, -0.4, 0.0, -1.2, 0.0, 0.8, 0.0])

        for idx, (x, y, z) in enumerate(path):
            # Build the desired end-effector pose
            T = sm.SE3(x, y, z) * sm.SE3.RPY(desired_rpy)

            # Solve IK from a decent initial guess
            sol = self.rtb_panda.ikine_LM(T, q0=q_ready)

            if sol.success:
                q_target = sol.q
                print(f"Waypoint {idx}: (x={x:.2f}, y={y:.2f}, z={z:.2f}) -> q={q_target}")

                # Command each joint
                for j in range(7):
                    if self.joint_nodes[j] is not None:
                        self.joint_nodes[j].setJointPosition(q_target[j])

                # Let the robot move for some steps
                # Also update our q_ready guess for next waypoint
                q_ready = q_target  
                for _ in range(20):
                    if self.step(self.timestep) == -1:
                        return
            else:
                print(f"IK failed at waypoint {idx}: (x={x:.2f}, y={y:.2f}, z={z:.2f})")

        print("Finished drawing path.")

    def generate_cu_path(self):
        """
        Return a list of (x, y, z) coordinates to form smaller "C" and "U" 
        within the Panda's typical workspace.
        """
        path = []
        # "C": half-circle around center ~ (0.5, 0, 0.3), radius 0.05
        cx, cy, cz = 0.65, 0.5, 0.7
        r_c = 0.1
        for theta in np.linspace(0, math.pi, 10):
            x = cx + r_c * math.cos(theta)
            y = cy + r_c * math.sin(theta)
            z = cz
            path.append((x, y, z))

        # Lift pen before drawing "U"
        path.append((0.6, 0.0, 0.7))

        # "U": partial circle from -90° to +90°, center at (0.6, 0, 0.3), radius 0.05
        ux, uy, uz = 0.65, 1, 0.7
        r_u = 0.1
        for theta in np.linspace(-math.pi / 2, math.pi / 2, 10):
            x = ux + r_u * math.cos(theta)
            y = uy + r_u * math.sin(theta)
            z = uz
            path.append((x, y, z))

        return path

def main():
    controller = PandaProtoSupervisor()
    controller.run()

if __name__ == "__main__":
    main()
