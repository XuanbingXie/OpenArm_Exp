import rebocap_ws_sdk
class Test:
    def __init__(self):
        # Initialize RoboCap SDK(Blender coordinate system, local rotation)
        rebocap_port = 7690
        self.sdk = rebocap_ws_sdk.RebocapWsSdk(
            coordinate_type=rebocap_ws_sdk.CoordinateType.BlenderCoordinate,
            use_global_rotation=False
        )
        self.sdk.set_pose_msg_callback(self.on_pose_data)
        self.sdk.set_exception_close_callback(self.on_exception_close)
        ret = self.sdk.open(rebocap_port)
        if ret != 0:
            self.cleanup()
            raise RuntimeError(f"Failed to connect to RoboCap (error code: {ret})")
    def on_pose_data(self, sdk, tran, pose24, static_index, ts):
        pass
    
    def on_exception_close(self, sdk, code):
        self.cleanup()

    def cleanup(self):
        if self.sdk:
            self.sdk.close()
            self.sdk.release()
            self.sdk = None

if __name__ == "__main__":
    test = Test()
    try:
        while True:
            pass
    except KeyboardInterrupt:
        test.cleanup()