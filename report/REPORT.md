# Báo cáo Day 6: QA nhãn 2D bằng LiDAR

> Trước khi nộp, hãy điền bốn trường thông tin học viên còn thiếu bên dưới.

- **Họ tên:** [ĐIỀN]
- **MSSV:** [ĐIỀN] (phải trùng với MSSV trong tên repo `<HoVaTen>-<MSSV>-Track4-Day21`)
- **Lớp:** [ĐIỀN]
- **Link repo:** [ĐIỀN]
- **Topic:** F — Hỗ trợ gán nhãn bằng LiDAR
- **Dataset:** `data/kitti_mini`
- **Các frame đã dùng:** 000001, 000004, 000007, 000008, 000009, 000010, 000011, 000012, 000015, 000016, 000019, 000021, 000023, 000025, 000031, 000032, 000043, 000048, 000049, 000061

## 1. Claim

Với xe Car có `truncated <= 0.1` và `occluded <= 1` ở khoảng cách camera `z >= 40 m`, median IoU của box tạo từ điểm LiDAR thấp hơn median IoU của box tạo bằng cách chiếu 8 góc 3D.

## 2. Evidence

Mỗi box dự đoán được so với bbox 2D KITTI bằng IoU liên tục; depth là `location[2]` của label. Median điểm là số raw LiDAR return nằm trong oriented 3D box.

| Tập / khoảng cách | n | IoU median, 8 góc | IoU median, điểm LiDAR | Median điểm trong 3D box |
|---|---:|---:|---:|---:|
| Sạch, <20 m | 20 | 0.976 | 0.808 | 759 |
| Sạch, 20–40 m | 14 | 0.968 | 0.715 | 127 |
| Sạch, >=40 m | 11 | 0.973 | 0.233 | 13 |
| Tất cả xe, <20 m | 32 | 0.971 | 0.791 | 890 |
| Tất cả xe, 20–40 m | 27 | 0.969 | 0.547 | 74 |
| Tất cả xe, >=40 m | 13 | 0.975 | 0.233 | 13 |

Claim được ủng hộ trong mẫu này: ở nhóm sạch >=40 m, IoU median là 0.233 so với 0.973. Kết quả mô tả 11 xe xa, không khẳng định ý nghĩa thống kê tổng quát.
Box từ điểm LiDAR có diện tích ở 10/11 xe sạch trong nhóm xa; frame `000009` là ví dụ box suy biến.

CSV theo từng object: `results/topic_f_object_iou.csv`; bảng tổng hợp: `results/topic_f_iou_by_distance.csv`; plot: `results/figures/topic_f_iou_by_distance.png`.

### CP3: kết quả theo mức che khuất

So sánh ba mức `occluded = 0/1/2` trên cùng dataset KITTI mini và class Car, giữ `truncated <= 0.1`. Sáu xe có `occluded = 3` không nằm trong ba mức này nên được loại khỏi bảng.

| Mức che khuất | n xe | IoU median, 8 góc 3D | IoU median, điểm LiDAR |
|---:|---:|---:|---:|
| 0 | 33 | 0.973 | 0.740 |
| 1 | 12 | 0.974 | 0.658 |
| 2 | 16 | 0.971 | 0.435 |

Khi mức che khuất tăng từ 0 lên 2, IoU median của box từ điểm LiDAR giảm từ 0.740 xuống 0.435; box chiếu từ 8 góc gần như ổn định, từ 0.973 xuống 0.971. Đây là xu hướng mô tả trong mẫu hiện có: phân bố khoảng cách giữa các mức che khuất không giống nhau, nên không thể kết luận che khuất là nguyên nhân duy nhất.

Đối chiếu riêng vùng `20–40 m` cho kết quả cùng chiều: IoU box LiDAR lần lượt là 0.789 / 0.508 / 0.485 với `n = 11 / 3 / 13`; IoU box 8 góc là 0.967 / 0.970 / 0.970. Nhóm `occluded = 1` chỉ có 3 xe nên kết quả nhóm này cần được xem thận trọng.

CSV tổng hợp: `results/topic_f_iou_by_occlusion.csv`; biểu đồ: `results/figures/topic_f_iou_by_occlusion.png`.

![So sánh hai box trên KITTI](../results/figures/topic_f_demo_000004.png)

![IoU theo khoảng cách](../results/figures/topic_f_iou_by_distance.png)

![IoU theo mức che khuất](../results/figures/topic_f_iou_by_occlusion.png)

![Projection baseline trên toàn ảnh](../results/figures/overlay_000004_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png)

## 3. Failure case

![Xe xa chỉ có một điểm LiDAR](../results/figures/fail_01_sparse_lidar_car_000009.png)

Frame `000009`, Car #2 ở `68.25 m`, không bị che khuất/cắt mép, có 1 điểm trong 3D box. Box từ điểm suy biến, không có diện tích và IoU bằng 0; box chiếu 8 góc có IoU 0.977. Lớp debug: **Preprocess / độ phủ cảm biến** — số return thưa ở xa không đủ tạo min/max box ổn định. Đây là mismatch cần xem xét, không tự chứng minh bbox 2D sai.

## 4. Khuyến nghị nếu triển khai thật

Dùng trong bước QA ngoại tuyến: hiển thị bbox 2D hiện có, box chiếu từ 3D và box từ điểm LiDAR cùng số điểm hỗ trợ để người gán nhãn đối chiếu. Box từ điểm LiDAR có thể chặt nhưng kém ổn định ở xa; không tự thay nhãn bằng box suy ra từ một vài điểm. Theo dõi IoU và số điểm theo khoảng cách, và để người xem quyết định khi hai nguồn không khớp.

## 5. Cách chạy lại

```bash
python -m src.topic_f_qa --data-root data/kitti_mini --out-dir results --demo-frame 000004 --demo-car-index 0 --failure-frame 000009 --failure-car-index 2
python -m starter.projection --data-root data/kitti_mini --frame 000004
```

Kiểm tra tái lập đã chạy lần hai bằng lệnh sau; cả ba CSV khớp hoàn toàn với lần chạy đầu. Thư mục kiểm tra tạm đã được xóa sau khi so sánh.

```bash
python -m src.topic_f_qa --data-root data/kitti_mini --out-dir results/repro_check --demo-frame 000004 --demo-car-index 0 --failure-frame 000009 --failure-car-index 2
python -c "from pathlib import Path; import filecmp; names=['topic_f_object_iou.csv','topic_f_iou_by_distance.csv','topic_f_iou_by_occlusion.csv']; ok=all(filecmp.cmp(Path('results')/n, Path('results/repro_check')/n, shallow=False) for n in names); print('GIỐNG HỆT' if ok else 'KHÁC NHAU'); assert ok"
```

## 6. Khai báo sử dụng AI

| Công cụ | Dùng cho việc gì | Kiểm chứng kết quả |
|---|---|---|
| OpenAI Codex | Phân tích yêu cầu, viết script, tạo bảng và ảnh từ dữ liệu | Đã chạy script trên 20 frame; đối chiếu 72 dòng object với bảng tổng hợp và kiểm tra frame `000009` có 1 điểm trong box. Người nộp cần đọc/hiểu code và tự chạy lại trước khi nộp. |
