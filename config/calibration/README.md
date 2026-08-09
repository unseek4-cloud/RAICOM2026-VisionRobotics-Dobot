# 现场标定文件放置位置

比赛现场用 `opencvCalibration.exe` 完成 Eye-In-Hand 标定后，将最终生成的 YAML
复制到本目录并命名为：

`CaliMatrixData.yaml`

程序读取其中的 `CamToTipTransform`、`cameraInternalMatrix` 和 `distCoeffs`。
仓库中的 `CaliMatrixData.example.yaml` 仅用于解析测试，**不能用于现场抓取**。

