# cvtrack head-to-head: drone video, 4 configurations

- source: `pexels_aerial_2034115.mp4`  frames: 200
- visdrone weights: `/home/hhh/Downloads/cv_tracking_demo/weights/visdrone_yolov8s.pt`
- osnet weights: `/home/hhh/Downloads/cv_tracking_demo/weights/osnet_x0_25_msmt17.pth.tar`

| configuration | IDs | mean_len | total rows | med dets/frame | max_frame |
|---|---:|---:|---:|---:|---:|
| MOG2 (v4 fallback) | 180 | 21.8 | 3924 | 20 | 199 |
| COCO yolov8s (no ReID) | 417 | 7.6 | 3161 | 16 | 199 |
| VisDrone yolov8s (no ReID) | 218 | 21.1 | 4600 | 24 | 199 |
| VisDrone yolov8s + ReID(OSNet MSMT17) | 218 | 21.1 | 4600 | 24 | 199 |
