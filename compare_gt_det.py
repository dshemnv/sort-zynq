import cv2
import argparse
import glob
import pathlib
import os
from collections import namedtuple


detection = namedtuple("Detection", ["frame", "xl", "yl", "w", "h"])


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seq", help="path to sequence to show", type=str)
    parser.add_argument("--gt", help="ground truth detections", type=str)
    parser.add_argument("--det", help="actual detections", type=str)
    return parser.parse_args()


def parse_detections(file):
    detections = []
    with open(file, "r") as f:
        for line in f:
            frame, _id, xl, yl, w, h, _, _, _, _ = line.rstrip().split(",")
            detections.append(
                detection(
                    int(frame),
                    round(float(xl)),
                    round(float(yl)),
                    round(float(w)),
                    round(float(h)),
                )
            )
    return detections


if __name__ == "__main__":
    args = get_args()
    seq_name = os.path.basename(args.seq)

    # Load images paths
    imgs = sorted(glob.glob(os.path.join(args.seq, "img1", "*.jpg")))

    if args.gt:
        gt_data = parse_detections(args.gt)
    if args.det:
        det_data = parse_detections(args.det)

    # if args.gt and args.det:
    #     if gt_data[-1].frame != det_data[-1].frame:
    #         print("Error, gt and det differ in number of frames")
    #         exit(1)
    gt_flag = True
    det_flag = True
    pause = False
    gt_bb = []
    det_bb = []
    index_imgs = enumerate(imgs, start=1)
    while True:
        if not pause:
            try:
                i, img = next(index_imgs)
            except StopIteration:
                break
        if args.gt:
            gt_bb = list(filter(lambda x: x.frame == i, gt_data))
        if args.det:
            det_bb = list(filter(lambda x: x.frame == i, det_data))

        image = cv2.imread(img)

        for gt in gt_bb:
            tl = (gt.xl, gt.yl)
            br = (gt.xl + gt.w, gt.yl + gt.h)
            if gt_flag:
                cv2.rectangle(image, tl, br, (255, 0, 0), 2)

        for det in det_bb:
            tl = (det.xl, det.yl)
            br = (det.xl + det.w, det.yl + det.h)
            if det_flag:
                cv2.rectangle(image, tl, br, (0, 0, 255), 2)

        resized = cv2.resize(image, (1240, 768), cv2.INTER_AREA)
        resized = cv2.putText(
            resized, str(i), (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255)
        )

        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        elif k == ord("g"):
            gt_flag = not gt_flag
        elif k == ord("d"):
            det_flag = not det_flag
        elif k == ord("p"):
            pause = not pause

        cv2.imshow(seq_name, resized)

    cv2.destroyAllWindows()


# Load gt data and det data

# check same nb of frames

# launch video at 30 fps
# Add buttons to slow, speed, stop or reset fps
# loop video
