import cv2
import numpy as np

import Kalman


class Tracker:
    def __init__(self, roi_scale_weight=0.8, response_thres=0.9, apce_thres=0.9,
                 keep_hist_cnt=30, learning_rate=0.012, min_scale=0.95, max_scale=1.05, scale_step=0.05, debug=False):
        # 分类器的超参数
        self.padding = 2.5  # 生成高斯矩阵的padding
        self.input_size = 256
        self.sigma = 0.6
        self.lambda_r = 0.0001
        self.update_rate = learning_rate
        # 多尺度预测参数
        self.min_roi_scale = min_scale
        self.max_roi_scale = max_scale
        self.roi_scale_step = scale_step
        self.roi_scale_weight = roi_scale_weight
        # HOG特征参数。windowSize是检测窗的大小
        self.block_size = (8, 8)
        self.block_stride = (4, 4)
        self.cell_size = (4, 4)
        self.n_bins = 9
        # 多特征融合参数
        self.hog_ratio = 1
        self.lbp_ratio = 0
        self.color_ratio = 0
        # 特征提取参数
        self._scale_w = 0
        self._scale_h = 0
        self._window_size = (0, 0)
        self.feature_cnt = 1
        self._features = [0 for _ in range(self.feature_cnt)]
        self._alpha = [0 for _ in range(self.feature_cnt)]
        # 置信度参数
        self._hist_apce = np.zeros(keep_hist_cnt)
        self._hist_response = np.zeros(keep_hist_cnt)
        self._hist_index = 0
        self.keep_hist_cnt = keep_hist_cnt
        self.response_thres = response_thres
        self.apce_thres = apce_thres
        # 其他参数
        self.correct_thres = 0
        self._target_pos = None  # format: (center x, center y, w, h)
        self.kalman = None
        self.debug = debug

    @staticmethod
    def limitRoi(img, rect):
        h_limit, w_limit, _ = img.shape
        nw, nh = rect[2], rect[3]
        if rect[0] + rect[2] // 2 > w_limit:
            nw = w_limit - rect[0]
        if rect[0] - rect[2] // 2 < 0:
            nw = rect[0]
        if rect[1] + rect[3] // 2 > h_limit:
            nh = h_limit - rect[1]
        if rect[1] - rect[3] // 2 < 0:
            nh = rect[1]
        return nw, nh

    def getHOGFeatures(self, img):
        w, h = self._window_size
        w_stride, h_stride = self.block_stride
        w = w // w_stride - 1
        h = h // h_stride - 1
        hog = cv2.HOGDescriptor(self._window_size, self.block_size, self.block_stride, self.cell_size, self.n_bins)
        hist = hog.compute(img, winStride=self._window_size, padding=(0, 0))
        return hist.reshape(w, h, 36).transpose(2, 1, 0)

    def getFeatures(self, img, pos):
        # 对矩形框做2.5倍的Padding处理
        cx, cy, w, h = pos
        w = int(w * self.padding) // 2 * 2
        h = int(h * self.padding) // 2 * 2
        w, h = self.limitRoi(img, (cx, cy, w, h))
        x = int(cx - w // 2)
        y = int(cy - h // 2)
        # 获取目标区域并提取HOG特征
        target_img = img[y: y + h, x: x + w, :]
        # cv2.imshow("padding", target_img)
        # cv2.waitKey(100)
        target_img = cv2.resize(target_img, self._window_size)
        hogFeature = self.getHOGFeatures(target_img) * self.hog_ratio
        feature = hogFeature
        # lbpFeature = self.getLBPFeatures(target_img, hogFeature.shape[1], hogFeature.shape[2])
        # cnFeature = self.getCNFeatures(target_img) * self.color_ratio
        # feature = np.concatenate((hogFeature, lbpFeature), axis=0)
        # 更新参数
        fc, fh, fw = feature.shape
        self._scale_h = fh / h
        self._scale_w = fw / w
        # 构造hann矩阵
        hann2t, hann1t = np.ogrid[0:fh, 0:fw]
        hann1t = 0.5 * (1 - np.cos(2 * np.pi * hann1t / (fw - 1)))
        hann2t = 0.5 * (1 - np.cos(2 * np.pi * hann2t / (fh - 1)))
        hann2d = hann2t * hann1t
        feature = feature * hann2d
        return np.array(feature)

    @staticmethod
    def plotFeatures(features):
        c, h, w = features.shape
        features = features.reshape(2, 2, 9, h, w).sum(axis=(0, 1))
        grid = 16
        hgrid = grid // 2
        img = np.zeros((h * grid, w * grid))

        for i in range(h):
            for j in range(w):
                for k in range(9):
                    x = int(10 * features[k, i, j] * np.cos(np.pi / 9 * k))
                    y = int(10 * features[k, i, j] * np.sin(np.pi / 9 * k))
                    cv2.rectangle(img=img, pt1=(j * grid, i * grid), pt2=((j + 1) * grid, (i + 1) * grid),
                                  color=(255, 255, 255))
                    x1 = j * grid + hgrid - x
                    y1 = i * grid + hgrid - y
                    x2 = j * grid + hgrid + x
                    y2 = i * grid + hgrid + y
                    cv2.line(img=img, pt1=(x1, y1), pt2=(x2, y2), color=(255, 255, 255), thickness=1)
                cv2.imshow("img", img)
                cv2.waitKey(10)

    def gaussian_peak(self, w, h):
        output_sigma = 0.125
        sigma = np.sqrt(w * h) / self.padding * output_sigma
        syh, sxh = h // 2, w // 2

        y, x = np.mgrid[-syh:-syh + h, -sxh:-sxh + w]
        x = x + (1 - w % 2) / 2.
        y = y + (1 - h % 2) / 2.
        g = 1. / (2. * np.pi * sigma ** 2) * np.exp(-((x ** 2 + y ** 2) / (2. * sigma ** 2)))
        return g

    @staticmethod
    def kernelCorrelation(x1, x2, sigma):
        fx1 = np.fft.fft2(x1)
        fx2 = np.fft.fft2(x2)
        tmp = np.conj(fx1) * fx2
        idft_rbf = np.fft.fftshift(np.fft.ifft2(np.sum(tmp, axis=0)))
        d = np.sum(x1 ** 2) + np.sum(x2 ** 2) - 2 * idft_rbf
        k = np.exp(-1 / sigma ** 2 * np.abs(d) / d.size)
        return k

    def train(self, x, y, sigma, lambda_r):
        k = self.kernelCorrelation(x, x, sigma)
        return np.fft.fft2(y) / (np.fft.fft2(k) + lambda_r)

    def detect(self, x, z, sigma, alpha):
        k = self.kernelCorrelation(x, z, sigma)
        # 傅里叶逆变换的实部
        return np.real(np.fft.ifft2(alpha * np.fft.fft2(k)))

    # pos format: (center_x, center_y, w, h)
    def addSample(self, sample, pos):
        # get window size
        scale = self.input_size / max(pos[3], pos[2])
        temp_scale = self.cell_size[0]
        nh = int(pos[3] * scale) // temp_scale * temp_scale + temp_scale
        nw = int(pos[2] * scale) // temp_scale * temp_scale + temp_scale
        self._window_size = (nw, nh)
        self._target_pos = pos
        self.kalman = Kalman.KalmanFilter(pos)

        feature = self.getFeatures(sample, pos)
        y = self.gaussian_peak(feature.shape[2], feature.shape[1])
        self._alpha[0] = self.train(feature, y, self.sigma, self.lambda_r)
        self._features[0] = feature

        # # get hog feature
        # hog_features = self.getFeatures(sample, pos)
        # y = self.gaussian_peak(hog_features.shape[2], hog_features.shape[1])
        # self.alpha[0] = self.train(hog_features, y, self.sigma, self.lambda_r)
        # self.features[0] = hog_features

        # # get lbp feature
        # lbp_features = self.getFeatures(sample, pos, type="lbp")
        # y = self.gaussian_peak(lbp_features.shape[1], lbp_features.shape[2])
        # self.alpha[1] = self.train(lbp_features, y, self.sigma, self.lambda_r)
        # self.features[1] = lbp_features

    @staticmethod
    def calcAPCE(response):
        h, w = response.shape
        maxRes = np.max(response)
        minRes = np.min(response)
        resMinusMin = (response - minRes).reshape(h * w, 1)
        for i in range(h * w):
            resMinusMin[i] = resMinusMin[i] ** 2
        avgRes = np.mean(resMinusMin)
        return (maxRes - minRes) ** 2 / avgRes

    def getPrediction(self, img, roi, temp_features, temp_alpha, fixed=False):
        cx, cy, w, h = roi
        max_response = -1
        apce = -1
        best_features = temp_features
        dx, dy, best_w, best_h = (0, 0, w, h)

        if fixed:
            min_scale = 1
            max_scale = 1
            step = 0
        else:
            min_scale = self.min_roi_scale
            max_scale = self.max_roi_scale
            step = self.roi_scale_step

        # 对不同的scale计算response
        for x_scale in np.arange(min_scale, max_scale, step):
            for y_scale in np.arange(min_scale, max_scale, step):
                roi = map(int, (cx, cy, w * x_scale, h * y_scale))
                roi = list(roi)

                new_features = self.getFeatures(img, roi)
                response = self.detect(temp_features, new_features, self.sigma, temp_alpha)
                rh, rw = response.shape
                idx = np.argmax(response)
                # 最大响应值
                res = np.max(response)

                # 限制追踪框增大面积
                if x_scale * y_scale > 1:
                    res *= self.roi_scale_weight

                if res > max_response:
                    max_response = res
                    dx = int((idx % rw - rw / 2) * self.block_stride[0] / x_scale)
                    dy = int((idx / rw - rh / 2) * self.block_stride[1] / y_scale)
                    best_w = int(w * x_scale)
                    best_h = int(h * y_scale)
                    best_features = new_features
                    # APCE
                    apce = self.calcAPCE(response)
                    # PSR

        return dx, dy, best_w, best_h, best_features, max_response, apce

    @staticmethod
    def getShiftedRect(pos, direction):
        cx, cy, w, h = pos
        if direction == 0:
            return cx - w // 2, cy, w, h
        elif direction == 1:
            return cx + w // 2, cy, w, h
        elif direction == 2:
            return cx, cy - h // 2, w, h
        else:
            return cx, cy + h // 2, w, h

    # return format: (left-top x, left-top y, w, h)
    def findTarget(self, img):
        cx, cy, w, h = self._target_pos
        img_h, img_w, _ = img.shape

        # 正特征提取
        guessPos = self.kalman.predict()
        dx, dy, best_w, best_h, best_features, max_response, apce \
            = self.getPrediction(img, guessPos, self._features[0], self._alpha[0], fixed=False)

        best_pos = (cx + dx, cy + dy, best_w, best_h)

        validResponse = max_response != -1
        validMaxRes = max_response > np.mean(self._hist_response) * self.response_thres
        validApce = apce > np.mean(self._hist_apce) * self.apce_thres

        if validResponse and validMaxRes and validApce:
            # 更新target_pos
            self._target_pos = best_pos

            # 更新features和参数
            self._features[0] = self._features[0] * (1 - self.update_rate) + best_features * self.update_rate
            y = self.gaussian_peak(best_features.shape[2], best_features.shape[1])
            new_alpha_f = self.train(best_features, y, self.sigma, self.lambda_r)
            self._alpha[0] = self._alpha[0] * (1 - self.update_rate) + new_alpha_f * self.update_rate

            # 更新历史响应值与apce值
            self._hist_index = (self._hist_index + 1) % self.keep_hist_cnt
            self._hist_response[self._hist_index] = max_response
            self._hist_apce[self._hist_index] = apce

            # 更新卡尔曼滤波器
            self.kalman.correct(best_pos)

            p = self._target_pos
            return p[0] - p[2]//2, p[1] - p[3]//2, p[2], p[3]
        else:
            return None

    # possible_area format: (left-top x, left-top y, w, h)
    # return format: (left-top x, left-top y, w, h)
    def correct_target(self, img, possible_areas):
        max_response = -1
        best_features = self._features
        best_possibility = None
        # 对不同的scale计算response
        for possible_roi in possible_areas:
            possible_roi = (possible_roi[0] + possible_roi[2]//2, possible_roi[1] + possible_roi[3]//2,
                            possible_roi[2], possible_roi[3])
            new_features = self.getFeatures(img, possible_roi)
            response = self.detect(self._features[0], new_features, self.sigma, self._alpha)
            res = np.max(response)
            if res > max_response:
                max_response = res
                best_possibility = possible_roi
                best_features = new_features

        if max_response > self.correct_thres:
            # 更新target_pos
            self._target_pos = best_possibility

            # 更新features和参数
            self._features[0] = self._features[0] * (1 - self.update_rate) + best_features * self.update_rate
            y = self.gaussian_peak(best_features.shape[2], best_features.shape[1])
            new_alpha_f = self.train(best_features, y, self.sigma, self.lambda_r)
            # self.alpha = self.alpha * (1 - self.update_rate) + new_alpha_f * self.update_rate
            self._alpha[0] = new_alpha_f

            p = self._target_pos
            return p[0] - p[2] // 2, p[1] - p[3] // 2, p[2], p[3]
        else:
            return None


if __name__ == "__main__":
    pass
