import io

import matplotlib.pyplot as plt
import tf_transformations as tr

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from crx_kinematics.utils.geometry import *


def create_transforms(T_list, T_listsol, T_R0_tool, T_R0_plane, frame_names, stamp):
    def make_tfstamped(frame_id, child_frame_id, T):
        trans = tr.translation_from_matrix(T)
        quat = tr.quaternion_from_matrix(T)
        return TransformStamped(
            header=Header(frame_id=frame_id, stamp=stamp),
            child_frame_id=child_frame_id,
            transform=Transform(
                translation=Vector3(x=trans[0], y=trans[1], z=trans[2]),
                rotation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
            ),
        )

    transforms = [
        make_tfstamped(frame_names[i], frame_names[i + 1], T) for i, T in enumerate(T_list)
    ]
    transforms.extend(
        [
            make_tfstamped(
                "R0" if i == 0 else "sol_" + frame_names[i],
                "sol_" + frame_names[i + 1],
                T,
            )
            for i, T in enumerate(T_listsol)
        ]
    )

    transforms.append(make_tfstamped("R0", "desired_pose", T_R0_tool))
    transforms.append(make_tfstamped("R0", "plane", T_R0_plane))

    return transforms


def create_marker_array(circle_candidate, all_circle_evaluations):
    markers = circle_candidate_markers(circle_candidate)
    markers.extend(circle_line_strips(all_circle_evaluations))
    for i, m in enumerate(markers):
        m.id = i

    return MarkerArray(markers=markers)


def circle_candidate_markers(ce):
    def matrix_to_pose(matrix: np.ndarray) -> Pose:
        trans = tr.translation_from_matrix(matrix)
        quat = tr.quaternion_from_matrix(matrix)

        return Pose(
            translation=Point(x=trans[0], y=trans[1], z=trans[2]),
            rotation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )

    markers = [
        Marker(
            header=Header(frame_id="R0"),
            ns="candidate_O4",
            type=Marker.SPHERE,
            pose=Pose(position=numpy_to_point(ce.O4)),
            scale=Vector3(x=0.02, y=0.02, z=0.02),
            color=ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.5),
        ),
        Marker(
            header=Header(frame_id="plane"),
            ns="candidate_O0O3O4_plane",
            pose=matrix_to_pose(ce.T_R0_plane),
            type=Marker.CYLINDER,
            scale=Vector3(x=2 * ce.dist_O4, y=2 * ce.dist_O4, z=0.001),
            color=ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),
        ),
    ]
    markers.extend(
        [
            Marker(
                header=Header(frame_id="R0"),
                ns="candidate_Z4Z5_arrows",
                type=Marker.ARROW,
                scale=Vector3(x=0.005, y=0.02, z=0.02),
                color=ColorRGBA(
                    r=0.0 if close_to_zero else 1.0,
                    g=1.0 if close_to_zero else 0.0,
                    b=0.0,
                    a=0.5,
                ),
                points=[numpy_to_point(start), numpy_to_point(end)],
            )
            for start, end, close_to_zero in [
                (ce.O4 + ce.Z5fulllength, ce.O4, False),
                (ce.O3UP, ce.O4, np.abs(ce.dot_product_up) < 0.1),
                (ce.O3DOWN, ce.O4, np.abs(ce.dot_product_down) < 0.1),
            ]
        ]
    )
    markers.extend(
        [
            Marker(
                header=Header(frame_id="R0"),
                ns="candidate_O3UP_O3DOWN",
                type=Marker.SPHERE,
                pose=Pose(position=numpy_to_point(O)),
                scale=Vector3(x=0.02, y=0.02, z=0.02),
                color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5),
            )
            for O in [ce.O3UP, ce.O3DOWN]
        ]
    )

    return markers


def circle_line_strips(circle_evaluations):
    O4_pts = [numpy_to_point(ce.O4) for ce in circle_evaluations]
    O3UP_pts = [numpy_to_point(ce.O3UP) for ce in circle_evaluations]
    O3DOWN_pts = [numpy_to_point(ce.O3DOWN) for ce in circle_evaluations]

    return [
        Marker(
            header=Header(frame_id="R0"),
            ns=ns,
            type=Marker.LINE_STRIP,
            scale=Vector3(x=0.002, y=0.002, z=0.002),
            color=color,
            points=pts,
        )
        for pts, ns, color in [
            (O4_pts, "line_strip_O4", ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)),
            (O3UP_pts, "line_strip_O3UP_O3DOWN", ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)),
            (O3DOWN_pts, "line_strip_O3UP_O3DOWN", ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)),
        ]
    ]


def numpy_to_point(arr):
    return Point(x=arr[0], y=arr[1], z=arr[2])


def make_plot_img(sample_signal_up, sample_signal_down, zeros, circle_evaluations, i):

    fig, ax = plt.subplots()
    x = np.linspace(0, 2 * np.pi, 360)
    ax.plot(x, sample_signal_up, color="r", label="Z5Z4UP")
    ax.plot(x, sample_signal_down, color="b", label="Z5Z4DOWN")
    ax.axhline(y=0.0, color="black", linestyle="-")
    ax.axvline(x=np.radians(i), color="black", linestyle="-")
    for z in zeros:
        ax.axvline(x=circle_evaluations[z].q, color="black", linestyle="--")
    ax.set_title("Figure 7: Sample signal dot products")
    ax.legend()
    fig.canvas.draw()

    io_buf = io.BytesIO()
    fig.savefig(io_buf, format="raw")
    io_buf.seek(0)
    image_array = np.reshape(
        np.frombuffer(io_buf.getvalue(), dtype=np.uint8),
        newshape=(int(fig.bbox.bounds[3]), int(fig.bbox.bounds[2]), -1),
    )
    io_buf.close()
    plt.close(fig)

    return image_array
