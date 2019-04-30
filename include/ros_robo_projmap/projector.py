import gl_projector
import numpy as np
import time
import matplotlib.pyplot as plt

class Projector():
    def __init__(self, calibration_matrix, x_res, y_res, proj_x_res=1366, proj_y_res=768, 
            entire=False, monitor=-1):

        inds = np.indices([y_res, x_res], dtype=np.float32).transpose([1, 2, 0])[..., ::-1]

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
        print("Using calibration matrix:")
        print(M)
        self.calibration_matrix = np.ascontiguousarray(M.astype(np.float32))
        gl_projector.start(self.calibration_matrix, inds, x_res, y_res, proj_x_res, proj_y_res, monitor)

    def draw_frame(self, rgb, depth):
        depth_f = depth.astype(np.float32)
        depth_f[depth_f == 0] = np.inf
        rgb_norm = rgb.astype(np.float32) / 255.0

        return gl_projector.draw_frame(depth_f, rgb_norm)

    def stop(self):
        gl_projector.stop()


if __name__ == '__main__':
    fps = 60
    width, height = 1920, 1080

    mvp = np.array([
        [1/1366.,  0,    0,  -0.5],
        [0,  -1/768.,    0,  0.5],
        [0,  0,          0,  0],
        [0.0, 0.0,      0,  0.5]
    ], dtype=np.float32)

    # mvp = np.array([
    #     [-1.06876169e-05, -1.21751787e-07,  6.55230630e-03,  2.29022865e-01],
    #     [-1.90532596e-07, -1.07657001e-05,  4.48697266e-03, -9.73388568e-01],
    #     [-2.27919223e-10, -4.21394547e-10, -5.50405379e-06, -3.96796793e-04],
    #     [0.0,              0.0,             0.0,             0.0           ]
    # ], dtype=np.float32)

    print(mvp)

    p = Projector(mvp, x_res=width, y_res=height, proj_x_res=1366, proj_y_res=768, entire=True)

    depth = np.ones([height, width], dtype=np.float32)

    for i in range(500):
        rgb = 255*np.random.rand(height, width, 3)
        start = time.time()
        if p.draw_frame(rgb, depth):
            break
        time.sleep(max(0, 1. / fps - (time.time() - start)))
