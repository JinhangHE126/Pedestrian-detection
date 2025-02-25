import cv2
import Detect
import Track


if __name__ == "__main__":
    # video_path是需要检测的视频路径
    # model_path是模型onnx文件的路径
    video_path = None
    model_path = None
    model_input_size = 416
    debug = False

    capture = cv2.VideoCapture(video_path)
    init_detect = True
    target_pos = None
    detector = Detect.Detector(input_size=model_input_size, model_path=model_path)
    tracker = Track.Tracker(debug=debug, learning_rate=0.012, roi_scale_weight=1, response_thres=0.5, apce_thres=0.5)
    lost_target = False

    if capture.isOpened():
        while True:
            ret, img = capture.read()
            if not ret:
                break # end of video
            if init_detect:
                target_pos = detector.findTarget(img, showResult=True)
                tracker.addSample(img, target_pos)
                init_detect = False
            elif lost_target:
                class_ids, confidences, boxes = detector.findPedestrian(img)
                # detector.plotResult(img, class_ids, boxes)
                target_pos = tracker.correct_target(img, boxes)
                if target_pos is not None:
                    lost_target = False
            else:
                target_pos = tracker.findTarget(img)
                if target_pos is None:
                    lost_target = True
                    continue
                Detect.Detector.plotBox(img, target_pos, color=(255, 255, 0))
                # tracker.plotFeatures(tracker.features)
                key = cv2.waitKey(1)
                if key & 0xFF == ord(' '):
                    cv2.waitKey(0)
                elif key & 0xFF == 27:
                    break
    else:
        print("failed to open the video")

    capture.release()
    cv2.destroyAllWindows()
