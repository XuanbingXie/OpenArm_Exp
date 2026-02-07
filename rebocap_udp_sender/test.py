import rebocap_ws_sdk
import gc, time

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
        self.pose = None
        self.last_pose = None

    def on_pose_data(self, sdk, tran, pose24, static_index, ts):
        # print(type(tran), type(pose24), type(static_index), type(ts))
        # self.last_pose = self.pose
        # self.pose = deepcopy(pose24)
        for i in range(24):
            pose24[i].clear()
        pose24.clear()
        tran.clear()
        # print("current:", self.pose[-11])
        # print("last:", self.last_pose[-11])

    def on_exception_close(self, sdk, code):
        self.cleanup()

    def cleanup(self):
        # rebocap_ws_sdk_ext.rebocap_ws_sdk_close(self.handle)
        # rebocap_ws_sdk_ext.rebocap_ws_sdk_release(self.handle)
        if self.sdk:
            self.sdk.close()
            # self.sdk.release()
            self.sdk = None
        

if __name__ == "__main__":
    test = Test()
    try:
        while True:
            time.sleep(1)
            # gc.collect()
            # # print(x, gc.get_stats())
            # # print(len(gc.get_objects()))
            # print(len(gc.get_objects(0)), len(gc.get_objects(1)), len(gc.get_objects(2)))
            # t = defaultdict(int)
            # m = defaultdict(str)
            # for obj in gc.get_objects():
            #     t[type(obj)] += 1
            #     m[type(obj)] += str(obj) + "\n"
            # gc.collect()
            # # print(t)
            # # pprint(gc.get_objects(2))
            # ## Find the max num of object, and print the type with the max num
            # max_num = 0
            # max_type = None
            # for obj_type, count in t.items():
            #     if count > max_num:
            #         max_num = count
            #         max_type = obj_type
            # print(m[max_type], file=open("max_type.txt", "w"))
            # # pprint(m)
            # print(f"Max object type: {max_type}, Count: {max_num}")
            # del t, m
            # gc.collect()

    except KeyboardInterrupt:
        test.cleanup()
        while True:
            print(len(gc.get_objects(0)), len(gc.get_objects(1)), len(gc.get_objects(2)))
            x = gc.collect()
            time.sleep(1)