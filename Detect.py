import cv2
from cv2 import dnn
import numpy as np


class Detector:
    def __init__(self,
                 model_path,
                 input_size,
                 colors=None,
                 class_desc=None,
                 conf_thres=0.3):
        if class_desc is None:
            class_desc = ["pedestrian", "partially visible"]
        if colors is None:
            colors = [(255, 255, 0), (0, 255, 255)]
        self.model = dnn.readNetFromONNX(model_path)
        self.input_size = input_size
        self.confidence_thres = conf_thres
        self.colors = colors
        self.class_desc = class_desc
        self.scale_factor = 1
        self.padding_dir = None
        self.padding_size = 0

    def resizeImg(self, img):
        h, w, _ = img.shape
        # letterbox resize
        scale = min(self.input_size / w, self.input_size / h)
        nw = int(w * scale)
        nh = int(h * scale)
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        img_back = np.ones((self.input_size, self.input_size, 3), dtype=np.uint8) * 128
        h_offset = (self.input_size - nh) // 2
        w_offset = (self.input_size - nw) // 2
        img_back[h_offset: h_offset + nh, w_offset: w_offset + nw, :] = img
        self.scale_factor = 1 / scale
        if nw < self.input_size:
            self.padding_dir = "w"
            self.padding_size = int(w_offset * self.scale_factor)
        else:
            self.padding_dir = "h"
            self.padding_size = int(h_offset * self.scale_factor)
        return img_back

    def preprocess(self, img):
        result = self.resizeImg(img)  # resize image
        # swap r and b channel
        # normalization to [-1, 1] for each channel
        # reshape input
        return cv2.dnn.blobFromImage(result, scalefactor=1 / 255, mean=[0, 0, 0], swapRB=True)
        # result = np.transpose(result, (2, 0, 1))
        # result = result / 255 # normalization
        # # reshape to (1, 3, img_size, img_size)
        # # 1 means 1 image
        # # 3 means 3 channels
        # return np.expand_dims(result, axis=0).astype(np.float32)

    def toBox(self, desc):
        x, y, w, h = desc[0], desc[1], desc[2], desc[3]
        left = int((x - 0.5 * w) * self.scale_factor)
        top = int((y - 0.5 * h) * self.scale_factor)
        if self.padding_dir == "w":
            left -= self.padding_size
        else:
            top -= self.padding_size
        width = int(w * self.scale_factor)
        height = int(h * self.scale_factor)
        return np.array([left, top, width, height])

    @staticmethod
    def plotBox(original_img, box, color, thickness=2):
        img = np.copy(original_img)
        cv2.rectangle(img, box, color, thickness)
        cv2.imshow("result", img)

    def plotResult(self, original_img, class_ids, boxes):
        img = np.copy(original_img)
        for i in range(len(class_ids)):
            box = boxes[i]
            cid = class_ids[i]
            color = self.colors[cid]
            cv2.rectangle(img, box, color, 2)
            cv2.rectangle(img, (box[0], box[1] - 20), (box[0] + 150, box[1]), color, -1)
            cv2.putText(img, self.class_desc[cid], (box[0] + 5, box[1] - 5), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
        cv2.imshow("result", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    @staticmethod
    def getSubset(s, indexes):
        subset = []
        for i in indexes:
            subset.append(s[i])
        return subset

    def findPedestrian(self, img):
        img_width, img_height, _ = img.shape

        self.model.setInput(self.preprocess(img))
        predictions = self.model.forward()[0].T

        class_ids = []
        confidences = []
        boxes = []

        # unwrap boxes
        for i in range(predictions.shape[0]):
            predict = predictions[i]
            scores = predict[4:]
            _, _, _, max_index = cv2.minMaxLoc(scores)
            class_id = max_index[1]
            confidence = max(scores)
            if confidence >= self.confidence_thres:
                class_ids.append(class_id)
                confidences.append(confidence)
                boxes.append(self.toBox([predict[i].item() for i in range(4)]))

        # nms
        final_indexes = []
        confidences = np.array(confidences)
        boxes = np.array(boxes)
        class_id_labels = np.unique(class_ids)
        for class_label in class_id_labels:
            i_set = [i for i, x in enumerate(class_ids) if x == class_label]
            result_indexes = list(dnn.NMSBoxes(boxes[i_set], confidences[i_set], 0.25, 0.45))
            final_indexes += self.getSubset(i_set, result_indexes)

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in final_indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    @staticmethod
    # box format: left-top x, left-top y, width, height
    def findLargest(img, confidences, boxes, conf_thres=0.5):
        # filter possible targets based on confidence
        valid_boxes = []
        for i in range(len(confidences)):
            if confidences[i] >= conf_thres:
                valid_boxes.append(boxes[i])

        # find largest from the remaining boxes
        target_box = None
        target_area = 0
        for i in range(len(valid_boxes)):
            b = valid_boxes[i]
            if b[2] * b[3] > target_area:
                target_box = b
                target_area = b[2] * b[3]

        # # cut the corresponding part from the image
        # if target_box[0] <= 0:
        #     target_box[0] = 0
        # elif target_box[1] <= 0:
        #     target_box[1] = 0

        result = img[target_box[1]: target_box[1] + target_box[3], target_box[0]: target_box[0] + target_box[2], :]

        # convert target_pos to (center_x, center_y, w, h) format
        target_box[0] = target_box[0] + target_box[2] // 2
        target_box[1] = target_box[1] + target_box[3] // 2
        return result, target_area, target_box

    def findTarget(self, img, showResult=False):
        class_ids, confidences, boxes = self.findPedestrian(img)
        if showResult:
            self.plotResult(img, class_ids, boxes)
        target, target_area, target_pos = self.findLargest(img, confidences, boxes)
        if showResult:
            cv2.imshow("target", target)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return target_pos


if __name__ == "__main__":
    test_img_path = "D:\\1\\PYCharm\\robot_go1\\pics\\3fa92d90-4372-4dcf-9cc2-3eaef3cbbb86.jpg"
    test_model_path = "D:\\1\\PYCharm\\robot_go1\\ultralytics\\train\\weights\\best.onnx"
    test_input_size = 416

    detector = Detector(model_path=test_model_path, input_size=test_input_size)
    orig_img = cv2.imread(test_img_path)
    detector.findTarget(orig_img, showResult=True)
