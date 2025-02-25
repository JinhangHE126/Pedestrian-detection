import numpy as np


class KalmanFilter:
    def __init__(self, init_box):
        # 状态向量
        self.state = np.array([
            init_box[0], init_box[1], init_box[2], init_box[3],
            0, 0, 0, 0
        ]).T
        # 状态转移
        self.A = np.array([
            [1, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ])
        self.default_A = np.array([
            [1, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ])
        # 状态观测
        self.H = np.eye(8)
        # 过程噪声协方差
        self.Q = np.eye(8) * 0.1
        # 观测噪声协方差
        self.R = np.eye(8) * 10
        # 状态估计协方差
        self.P = np.eye(8)
        # 控制输入
        self.B = None

    def predict(self):
        predict = np.dot(self.A, self.state)
        self.state = predict
        return predict[0: 4]

    def correct(self, newResult):
        predict = np.dot(self.A, self.state)

        P_prior_1 = np.dot(self.A, self.P)
        P_prior = np.dot(P_prior_1, self.A.T) + self.Q

        K1 = np.dot(P_prior, self.H.T)
        K2 = np.dot(np.dot(self.H, P_prior), self.H.T) + self.R
        K = np.dot(K1, np.linalg.inv(K2))

        Z = [0, 0, 0, 0, 0, 0, 0, 0]
        for i in range(4):
            Z[i] = newResult[i]
            Z[i + 4] = newResult[i] - self.state[i]
        X_posterior_1 = Z - np.dot(self.H, predict)
        self.state = predict + np.dot(K, X_posterior_1)

        P_posterior_1 = np.eye(8) - np.dot(K, self.H)
        self.P = np.dot(P_posterior_1, P_prior)
