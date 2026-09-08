"""Headless ROS integration test; no terrain demo, robot, or RViz required.

Source install/setup.bash, then run with /usr/bin/python3 and the executable path
as the first argument. Uses a separate, localhost-only ROS domain (default 167).
"""

import argparse
import math
import os
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from convex_plane_decomposition_msgs.msg import (
    PlanarRegion, PlanarTerrain, Point2d, Polygon2d, PolygonWithHoles2d,
)
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray


def polygon(points):
    return PolygonWithHoles2d(outer_boundary=Polygon2d(
        points=[Point2d(x=float(x), y=float(y)) for x, y in points]))


def terrain(width=2.0, angle=0.0, shape=None):
    message = PlanarTerrain()
    grid = message.gridmap
    grid.header.frame_id = "odom"
    grid.info.resolution = 1.0
    grid.info.length_x = grid.info.length_y = 2.0
    grid.info.pose.orientation.w = 1.0
    grid.layers = grid.basic_layers = ["elevation"]
    data = Float32MultiArray(data=[0.0] * 4)
    data.layout.dim = [
        MultiArrayDimension(label="column_index", size=2, stride=4),
        MultiArrayDimension(label="row_index", size=2, stride=2),
    ]
    grid.data = [data]
    half = width / 2.0
    edge = min(0.1, width / 8.0)
    region = PlanarRegion()
    region.plane_parameters.orientation.y = math.sin(angle / 2.0)
    region.plane_parameters.orientation.w = math.cos(angle / 2.0)
    region.boundary = polygon(shape or [(-half, -1), (half, -1), (half, 1), (-half, 1)])
    region.insets = [polygon([(-half + edge, -0.9), (half - edge, -0.9),
                              (half - edge, 0.9), (-half + edge, 0.9)])]
    region.bbox2d.min_x, region.bbox2d.max_x = -half, half
    region.bbox2d.min_y, region.bbox2d.max_y = -1.0, 1.0
    message.planar_regions = [region]
    return message


def find(markers, namespace):
    return next(marker for marker in markers.markers if marker.ns == namespace)


def validate(markers, margin, angle=0.0):
    assert all(marker.type != Marker.TEXT_VIEW_FACING for marker in markers.markers)
    rotation = np.array([[math.cos(angle), 0, math.sin(angle)],
                         [0, 1, 0], [-math.sin(angle), 0, math.cos(angle)]])

    def plane_points(marker):
        return np.array([[p.x, p.y, p.z] for p in marker.points]) @ rotation

    raw = plane_points(find(markers, "Selected convex region (before margin)"))[:-1, :2]
    feasible = plane_points(find(markers, "Actual constraint boundary"))[:-1, :2]
    assert len(raw) == 16
    edges = np.roll(raw, -1, axis=0) - raw
    normals = np.column_stack((-edges[:, 1], edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    center = raw.mean(axis=0)
    normals[np.sum(normals * (center - raw), axis=1) < 0] *= -1
    offsets = -np.sum(normals * raw, axis=1) - margin
    assert np.min(feasible @ normals.T + offsets) > -1e-7, "Vertex violates a half-space"
    # Every output edge must be supported by an active half-space.
    midpoints = (feasible + np.roll(feasible, -1, axis=0)) / 2.0
    assert np.max(np.min(np.abs(midpoints @ normals.T + offsets), axis=1)) < 1e-7
    fill = find(markers, "Actual constraint feasible set")
    assert fill.scale.x == fill.scale.y == fill.scale.z == 1.0
    assert len(fill.points) == 3 * len(feasible)
    for arrow in markers.markers:
        if arrow.ns != "Half-space inward normals":
            continue
        points = plane_points(arrow)[:, :2]
        vector = points[1] - points[0]
        edge = feasible[(arrow.id + 1) % len(feasible)] - feasible[arrow.id]
        assert abs(vector @ edge) < 1e-8, "Arrow is not a perpendicular normal"
        assert vector @ (feasible.mean(axis=0) - points[0]) > 0
    assert all(marker.header.frame_id == "odom" for marker in markers.markers)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable")
    parser.add_argument("--domain-id", default=167, type=int)
    parser.add_argument("--margin", default=0.05, choices=[0.0, 0.05], type=float)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    with tempfile.TemporaryDirectory(prefix="foot-placement-smoke-") as directory:
        os.environ["ROS_LOG_DIR"] = directory
        with open(os.path.join(directory, "node.log"), "w+") as log:
            process = subprocess.Popen([
                os.path.abspath(args.executable), "--ros-args", "-p", f"boundary_margin:={args.margin}",
                "-p", "republish_rate:=0.0"
            ], stdout=log, stderr=log)
            node = None
            try:
                rclpy.init()
                node = rclpy.create_node("foot_placement_smoke_test")
                received = []
                subscription = node.create_subscription(
                    MarkerArray, "/foot_placement_constraint_demo", received.append, 10)
                publisher = node.create_publisher(PlanarTerrain, "/planar_terrain", 1)
                clicks = node.create_publisher(PointStamped, "/clicked_point", 10)

                def wait(predicate, timeout=15.0):
                    end = time.monotonic() + timeout
                    while time.monotonic() < end:
                        assert process.poll() is None, "Visualizer exited"
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if predicate():
                            return
                    raise AssertionError("Timed out waiting for ROS output")

                def send(message):
                    received.clear()
                    publisher.publish(message)
                    wait(lambda: any(len(m.markers) > 1 for m in received))
                    return received[-1]

                def send_click(message):
                    received.clear()
                    clicks.publish(message)
                    wait(lambda: bool(received))

                wait(lambda: publisher.get_subscription_count() > 0 and clicks.get_subscription_count() > 0)
                output = send(terrain())
                validate(output, args.margin)
                print("PASS flat terrain: margin, half-spaces, triangle scale, inward normals", flush=True)

                output = send(terrain(angle=0.35))
                # Wait for the slope marker if an earlier timer message arrived first.
                wait(lambda: any(abs(p.z - 0.008) > 0.01 for p in find(
                    received[-1], "Selected convex region (before margin)").points))
                validate(received[-1], args.margin, angle=0.35)
                print("PASS slope: plane-to-world transform", flush=True)

                # Restore a centered seed before testing a narrow region.
                click = PointStamped()
                click.header.frame_id = "odom"
                send_click(click)
                output = send(terrain(width=0.08))
                validate(received[-1], 0.0)
                print("PASS narrow terrain: correct margin/fallback state", flush=True)

                concave = terrain(shape=[(-1, -1), (1, -1), (1, 0),
                                         (0, 0), (0, 1), (-1, 1)])
                concave.planar_regions[0].insets = [polygon([
                    (-0.9, -0.9), (0.9, -0.9), (0.9, -0.1),
                    (-0.1, -0.1), (-0.1, 0.9), (-0.9, 0.9)])]
                click.point.x, click.point.y = -0.5, 0.5
                send_click(click)
                send(concave)
                wait(lambda: abs(find(received[-1], "Nominal foothold").pose.position.y - 0.5) < 1e-8)
                validate(received[-1], args.margin)
                raw = find(received[-1], "Selected convex region (before margin)")
                assert not any(p.x > 1e-8 and p.y > 1e-8 for p in raw.points)
                print("PASS concave terrain: local convex region stays inside boundary", flush=True)

                holed = terrain()
                holed.planar_regions[0].boundary.holes = [polygon([
                    (-0.2, -0.2), (-0.2, 0.2), (0.2, 0.2), (0.2, -0.2)]).outer_boundary]
                holed.planar_regions[0].insets[0].holes = [polygon([
                    (-0.3, -0.3), (-0.3, 0.3), (0.3, 0.3), (0.3, -0.3)]).outer_boundary]
                click.point.x = click.point.y = 0.0
                send_click(click)
                send(holed)
                wait(lambda: abs(find(received[-1], "Nominal foothold").pose.position.y) < 1e-8)
                validate(received[-1], args.margin)
                projected = find(received[-1], "Projected foothold").pose.position
                # Point2d coordinates are serialized as float32.
                assert abs(max(abs(projected.x), abs(projected.y)) - 0.3) < 1e-7, str(projected)
                print("PASS hole: seed projected out of hole, valid convex constraints", flush=True)

                send(terrain())
                click.point.x = 1.4
                send_click(click)
                wait(lambda: abs(find(received[-1], "Nominal foothold").pose.position.x - 1.4) < 1e-8)
                assert abs(find(received[-1], "Projected foothold").pose.position.x - 0.9) < 1e-7
                validate(received[-1], args.margin)
                print("PASS click: reselect and project outside nominal point to inset", flush=True)

                click.header.frame_id = "wrong_frame"
                click.point.x = 9.0
                clicks.publish(click)
                send(terrain())
                assert abs(find(received[-1], "Nominal foothold").pose.position.x - 1.4) < 1e-8
                print("PASS mismatched frame: click rejected", flush=True)

                empty = terrain()
                empty.planar_regions = []
                publisher.publish(empty)
                wait(lambda: len(received[-1].markers) == 1 and received[-1].markers[0].action == Marker.DELETEALL)
                print("PASS empty terrain: stale markers cleared", flush=True)
                assert subscription is not None
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


if __name__ == "__main__":
    main()
