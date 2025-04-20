from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation


def ssa(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Converts euler angles to quaternion (x, y, z, w)."""
    rotation_matrix = Rotation.from_euler("XYZ", [roll, pitch, yaw]).as_matrix()
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()
    return quaternion


def quat_to_euler(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Converts quaternion (x, y, z, w) to euler angles."""
    rotation_matrix = Rotation.from_quat([x, y, z, w]).as_matrix()
    euler_angles = Rotation.from_matrix(rotation_matrix).as_euler("XYZ")
    return euler_angles


@dataclass(slots=True)
class PoseData:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def __add__(self, other: "PoseData") -> "PoseData":
        return PoseData(
            **{
                field.name: getattr(self, field.name) + getattr(other, field.name)
                for field in self.__dataclass_fields__.values()
            }
        )

    def __sub__(self, other: "PoseData") -> "PoseData":
        return PoseData(
            **{
                field.name: getattr(self, field.name) - getattr(other, field.name)
                for field in self.__dataclass_fields__.values()
            }
        )

    def __mul__(self, other: float) -> "PoseData":
        if isinstance(other, PoseData):
            return PoseData(
                **{
                    field.name: getattr(self, field.name) * getattr(other, field.name)
                    for field in self.__dataclass_fields__.values()
                }
            )
        return PoseData(
            **{
                field.name: getattr(self, field.name) * other
                for field in self.__dataclass_fields__.values()
            }
        )

    def as_rotation_matrix(self) -> np.ndarray:
        euler_angles = [self.roll, self.pitch, self.yaw]
        quat = euler_to_quat(*euler_angles)
        rotation_matrix = Rotation.from_quat(quat).as_matrix()
        return rotation_matrix


@dataclass(slots=True)
class TwistData:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    def __add__(self, other: "TwistData") -> "TwistData":
        return TwistData(
            **{
                field.name: getattr(self, field.name) + getattr(other, field.name)
                for field in self.__dataclass_fields__.values()
            }
        )

    def __sub__(self, other: "TwistData") -> "TwistData":
        return TwistData(
            **{
                field.name: getattr(self, field.name) - getattr(other, field.name)
                for field in self.__dataclass_fields__.values()
            }
        )

    def __mul__(self, other: float) -> "TwistData":
        if isinstance(other, TwistData):
            return TwistData(
                **{
                    field.name: getattr(self, field.name) * getattr(other, field.name)
                    for field in self.__dataclass_fields__.values()
                }
            )
        return TwistData(
            **{
                field.name: getattr(self, field.name) * other
                for field in self.__dataclass_fields__.values()
            }
        )


@dataclass(slots=True)
class State:
    pose: PoseData = field(default_factory=PoseData)
    twist: TwistData = field(default_factory=TwistData)

    def __add__(self, other: "State") -> "State":
        return State(pose=self.pose + other.pose, twist=self.twist + other.twist)

    def __sub__(self, other: "State") -> "State":
        return State(pose=self.pose - other.pose, twist=self.twist - other.twist)
