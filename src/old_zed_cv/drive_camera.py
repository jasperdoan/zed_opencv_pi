from controller_manager.zed_subscriber import ZEDSub
from rclpy.executors import SingleThreadedExecutor
from threading import Thread


class Camera(object):
    def __init__(self):
        self.zed_sub = ZEDSub()
        self.camera_executor = SingleThreadedExecutor()
        self.camera_executor.add_node(self.zed_sub)
        self.camera_thread = Thread(target=self.camera_executor.spin, daemon=True)
        self.camera_thread.start()

    def take_zed_snapshot(self):
        self.zed_sub.snapshot = True

    def get_zed_frame(self):
        return self.zed_sub.frame

    def close(self):
        self.camera_executor.shutdown()
        self.zed_sub.destroy_node()
        self.camera_thread.join()




