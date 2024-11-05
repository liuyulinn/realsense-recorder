import numpy as np
import pyrealsense2 as rs


class Observer:
    def __init__(self, align_to="depth"):
        self.pipe = rs.pipeline()
        profile = self.pipe.start()
        sensor = profile.get_device().query_sensors()[0]
        # sensor.set_option(rs.option.min_distance, 0)

        self.pc = rs.pointcloud()

        # print(profile.get_streams())
        depth, color = profile.get_streams()

        if align_to == "depth":
            self.align = rs.align(depth.stream_type())
            intrinsics = depth.as_video_stream_profile().get_intrinsics()
            K = np.array(
                [
                    [intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1],
                ]
            )
            self.Kinv = np.linalg.inv(K)
            self.K = K
        elif align_to == "color":
            self.align = rs.align(color.stream_type())
            intrinsics = color.as_video_stream_profile().get_intrinsics()
            K = np.array(
                [
                    [intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1],
                ]
            )
            self.Kinv = np.linalg.inv(K)
            self.K = K

        # print("depth intrinsics", depth.as_video_stream_profile().get_intrinsics())
        # print("infrared intrinsics", infrared.as_video_stream_profile().get_intrinsics())
        # print("color intrinsics", color.as_video_stream_profile().get_intrinsics())
        # print("depth to color", depth.get_extrinsics_to(color))
        # print(depth.get_extrinsics_to(infrared))

    @property
    def intrinsic_matrix(self):
        return self.K

    def get_rgb_depth(self):
        frames = self.pipe.wait_for_frames()
        frames = self.align.process(frames)

        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        # self.pc.map_to(depth)
        # H = depth.get_height()
        # W = depth.get_width()
        # points = self.pc.calculate(depth)
        # points = np.array(
        #     [[x for x in v] for v in np.asanyarray(points.get_vertices())]
        # )

        depth = np.array(depth.data) / 1000

        return depth, np.array(color.data)

    def get_frame(self):
        frames = self.pipe.wait_for_frames()
        frames = self.align.process(frames)

        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        H = depth.get_height()
        W = depth.get_width()

        # self.pc.map_to(depth)
        # points = self.pc.calculate(depth)
        # points = np.array(
        #     [[x for x in v] for v in np.asanyarray(points.get_vertices())]
        # )

        depth = np.array(depth.data) / 1000

        xy = np.stack(np.meshgrid(np.arange(W), np.arange(H)), axis=-1)
        xyz = np.concatenate((xy * depth[..., None], depth[..., None]), axis=-1)
        position = xyz @ self.Kinv.T

        # return position, np.array(color.data)
        return position, np.array(color.data)

    def get_frame_gl(self):
        position, color = self.get_frame()
        position[..., [1, 2]] *= -1
        return position, color

    def get_frame_ros(self):
        position, color = self.get_frame()
        position = position[..., [2, 0, 1]]
        position[..., [1, 2]] *= -1

        return position, color

    def get_images(self, take_picture=False) -> dict[str, np.ndarray]:
        if take_picture:
            frames = self.pipe.wait_for_frames()
            frames = self.align.process(frames)

            color = frames.get_color_frame()
            depth = frames.get_depth_frame()

            return {
                "rgb": np.array(color.data),
                "depth": np.array(depth.data) / 4000,
            }

        else:
            raise NotImplementedError

    def get_images_hw(self, take_picture=True) -> tuple[int, int]:
        if take_picture:
            frames = self.pipe.wait_for_frames()
            frames = self.align.process(frames)

            color = frames.get_color_frame()
            depth = frames.get_depth_frame()

            H = depth.get_height()
            W = depth.get_width()
            return H, W

        else:
            raise NotImplementedError
