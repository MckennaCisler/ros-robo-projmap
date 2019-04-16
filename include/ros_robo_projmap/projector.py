import gl_projector
import numpy as np
import time
import matplotlib.pyplot as plt

class Projector():
    def __init__(self, calibration_matrix, x_res, y_res, proj_x_res=1366, proj_y_res=768, entire=False, monitor=-1):
        self.inds = np.indices([y_res, x_res], dtype=np.float32).transpose([1, 2, 0])[..., ::-1]
        print('hello', entire)

        A = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 1, 0]
        ], dtype=np.float32)
        B = np.array([
            [1.0 / proj_x_res,    0,              0, 0],
            [0,                 1.0 / proj_y_res, 0, 0],
            [0,                 0,              1, 0],
            [0,                 0,              0, 0.5]
        ], dtype=np.float32)
        C = np.array([
            [1,  0,  0, -1],
            [0,  -1,  0,  1],
            [0,  0,  1,  0],
            [0,  0,  0,  1]
        ], dtype=np.float32)

        M = np.matmul(np.matmul(C, B), np.matmul(A, calibration_matrix))

        if entire:
            M = calibration_matrix

        M /= M[3, 3]
        print(M)
        self.calibration_matrix = np.ascontiguousarray(M.astype(np.float32))
        gl_projector.start(self.calibration_matrix, x_res, y_res, proj_x_res, proj_y_res, monitor)

    def draw_frame(self, rgb, depth):
        depth = np.expand_dims(depth, -1)
        coords = np.concatenate([
            self.inds * depth,
            depth,
            rgb / 255.0
        ], -1)
        return gl_projector.draw_frame(coords.astype(np.float32))

    def stop(self):
        gl_projector.stop()


if __name__ == '__main__':
    fps = 60
    width, height = 1920, 1080

    # mvp = np.array([
    #     [1/1366.,  0,    0,  -0.5],
    #     [0,  -1/768.,    0,  0.5],
    #     [0,  0,          0,  0],
    #     [0.0, 0.0,      0,  0.5]
    # ], dtype=np.float32)

    mvp = np.array([
        [-1.06876169e-05, -1.21751787e-07,  6.55230630e-03,  2.29022865e-01],
        [-1.90532596e-07, -1.07657001e-05,  4.48697266e-03, -9.73388568e-01],
        [-2.27919223e-10, -4.21394547e-10, -5.50405379e-06, -3.96796793e-04],
        [0.0, 0.0, 0.0, 0.0]
    ], dtype=np.float32)

    print(mvp)

    p = Projector(mvp, x_res=width, y_res=height, proj_x_res=1366, proj_y_res=768, entire=False)

    depth = 1000 * np.ones([height, width], dtype=np.float32)

    for i in range(500):
        rgb = 255*np.random.rand(height, width, 3)
        start = time.time()
        if p.draw_frame(rgb, depth):
            break
        time.sleep(max(0, 1. / fps - (time.time() - start)))
