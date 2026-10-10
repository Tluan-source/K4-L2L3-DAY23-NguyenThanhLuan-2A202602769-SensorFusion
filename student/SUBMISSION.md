# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Thành Luân
- MSSV: 2A202602769
- Email: 26ai.luannt2@vinuni.edu.vn
- Link repo (fork): https://github.com/Tluan-source/K4-L2L3-DAY23-NguyenThanhLuan-2A202602769-SensorFusion.git
- Commit hash nộp (`git rev-parse HEAD`): Sẽ là commit cuối cùng trên nhánh main

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: compare, [0, 198], training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord, 0
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision = 0.9701 (97.01%), recall = 0.7004 (70.04%), tp = 519, fp = 16, fn = 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1503 m, matches = 502, sum_sq_err = 11.3436 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1359 m, matches = 502, sum_sq_err = 9.2668 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  - Ở cả hai chế độ, số cặp ghép (`matches = 502`), số ghost tracks (`ghost_track_frames = 0`), số ground truth bị bỏ sót (`missed_gt_frames = 239`) và số confirmed tracks trung bình (`mean_confirmed_tracks = 2.5226`) là hoàn toàn như nhau. Điều này chứng minh kiến trúc **track-then-fuse** hoạt động đúng thiết kế: camera chỉ cập nhật trạng thái EKF mà không tham gia tạo, xác nhận hay xóa track (vòng đời track hoàn toàn do LiDAR quyết định).
  - Về độ chính xác vị trí: Chế độ fused đạt RMSE = 0.1359 m, tốt hơn so với chế độ LiDAR đơn lẻ (RMSE = 0.1503 m, giảm ~9.6% sai số), với tổng bình phương sai số vị trí giảm từ 11.3436 m² xuống 9.2668 m² (`rmse_fused - rmse_lidar = -0.0144 m <= 0.05 m`). Việc bổ sung đo lường góc/pixel từ camera giúp thu hẹp ma trận hiệp phương sai sai số $P$, làm ước lượng trạng thái 3D mịn và chính xác hơn mà không hề làm tracking xấu đi hay phát sinh ghost tracks.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
   - **LiDAR 3D:** Đo trực tiếp vị trí trong không gian xe $z = [x, y, z]^T \in \mathbb{R}^3$ (đơn vị mét). Ma trận nhiễu đo $R = \text{diag}(\sigma_{lidar\_x}^2, \sigma_{lidar\_y}^2, \sigma_{lidar\_z}^2) \in \mathbb{R}^{3 \times 3}$ (với $\sigma = 0.1$ m). Hàm đo $h(x)$ là tuyến tính, ma trận Jacobian $H$ kích thước $3 \times 6$ độc lập với trạng thái $x$ ($H = [I_3 \mid 0_3]$).
   - **Camera 2D:** Đo tọa độ pixel trên mặt phẳng ảnh $z = [u, v]^T \in \mathbb{R}^2$ (đơn vị pixel). Ma trận nhiễu đo $R = \text{diag}(\sigma_{cam\_i}^2, \sigma_{cam\_j}^2) \in \mathbb{R}^{2 \times 2}$ (với $\sigma = 5.0$ pixel). Hàm đo $h(x)$ phi tuyến theo mô hình pinhole chiếu phối cảnh sau khi đổi sang hệ tọa độ camera ($u = c_i - f_i \cdot y_s / x_s, v = c_j - f_j \cdot z_s / x_s$), đòi hỏi tính ma trận Jacobian $H \in \mathbb{R}^{2 \times 6}$ phụ thuộc vào trạng thái $x$ theo quy tắc dây chuyền.

2. Vì sao cần gating Mahalanobis trước khi gán?
   - Khoảng cách Mahalanobis $d^2 = \gamma^T S^{-1} \gamma$ tính khoảng cách giữa đo lường và dự báo có chuẩn hóa theo ma trận hiệp phương sai innovation $S = H P H^T + R$. Không giống như khoảng cách Euclidean đơn thuần chỉ đo khoảng cách hình học, Mahalanobis tính đến độ không chắc chắn (uncertainty ellipsoid) của track theo từng trục.
   - Gating bằng kiểm định $\chi^2$ (ngưỡng phân vị 0.995 theo bậc tự do của số chiều cảm biến) giúp loại bỏ ngay các đo lường ngoại lai, nhiễu nền hoặc các xe khác nằm ngoài vùng xác suất hợp lệ trước khi đưa vào ma trận chi phí. Điều này giúp thuật toán gán Greedy không gán nhầm đối tượng, ngăn ngừa phân kỳ bộ lọc và giảm tải tính toán.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   - Pipeline của lab là **track-then-fuse**: Chỉ duy trì một danh sách track duy nhất. Mỗi frame, EKF thực hiện predict một lần cho toàn bộ track hiện có, sau đó gán và cập nhật EKF với LiDAR (AssocL), rồi tiếp tục gán và cập nhật EKF với Camera (AssocC) trên chính các track đó.
   - Minh chứng trên log và mã nguồn `platform/fusion_lab/scripts/run_lab.py`:
     Trong vòng lặp xử lý mỗi frame (dòng 199–209):
     1. Gọi `KF.predict(track)` một lần cho toàn bộ track.
     2. Gọi `assoc.associate_and_update(manager, observations, KF, lidar_sensor)` để gán và update bằng LiDAR, đồng thời quản lý vòng đời (`manage_tracks`).
     3. Gọi `assoc.associate_and_update(manager, observations, KF, camera_sensor)` để gán và update bổ sung bằng Camera cho các track nằm trong FOV camera mà không can thiệp vòng đời.
     Dữ liệu thô của 2 cảm biến không bị gộp trước khi detection, mà được fuse ở tầng track thông qua 2 bước update EKF liên tiếp.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   - Khi camera bị lệch góc xoay hoặc vị trí tịnh tiến (extrinsic calibration error), hình chiếu dự báo $h(x)$ sẽ bị dịch chuyển có hệ thống so với vị trí pixel thực tế $z$.
   - Triệu chứng:
     - Innovation $\gamma = z - h(x)$ sẽ mang độ lệch có hệ thống (systematic bias, kỳ vọng khác 0 thay vì dao động ngẫu nhiên quanh 0).
     - Khoảng cách Mahalanobis $d^2$ tăng vọt. Nếu độ lệch vượt ngưỡng $\chi^2$, đo lường camera sẽ bị từ chối hàng loạt (gated out), dẫn đến camera không update được vào track.
     - Nếu độ lệch nằm sát ngưỡng và lọt qua gate, EKF update sẽ kéo trạng thái track lệch khỏi vị trí thật, làm tăng sai số vị trí 3D và khiến RMSE tăng cao.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   - **Cần sensor tường minh ở frame rỗng:** Ngay cả khi `meas_list` rỗng (không có đo lường nào trong frame), tracker vẫn bắt buộc phải gọi `manager.manage_tracks(...)` để hoàn tất vòng đời. Nếu là lượt LiDAR, các track nằm trong tầm nhìn LiDAR mà không ghép được với đo lường (miss in FOV) phải bị trừ điểm score ($-1/\text{window}$); các track có score $\le 0$ hoặc phương sai $P > \text{max\_P}$ phải bị xóa. Nếu là lượt camera, hàm quản lý không được trừ score hay xóa track. Do đó, tham số `sensor` tường minh là bắt buộc để hàm quản lý phân biệt được modality đang thực thi.
   - **Lý do LiDAR quyết định vòng đời còn Camera chỉ update EKF:** LiDAR đo trực tiếp tọa độ 3D với trường nhìn 360 độ quanh xe và độ sâu chính xác cao, đủ cơ sở tin cậy để xác định sự xuất hiện hay biến mất của vật thể. Camera trong lab chỉ quan sát hướng FRONT và đo góc 2D (không có độ sâu trực tiếp); nếu cho phép camera trừ điểm hay xóa track, các xe đi ra ngoài FOV camera (hoặc ở hai bên hông/phía sau xe) sẽ bị xóa oan dù LiDAR vẫn đang thấy rõ.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - **Điều kiện xác nhận (confirmed):** Track có điểm tồn tại `score > confirmed_threshold` (ngưỡng 0.8). Mỗi lần gán thành công với LiDAR (hit), track được cộng $+1/\text{window} = +1/6$ (tối đa 1.0). Sau ít nhất 5 frame hit liên tiếp ($5/6 \approx 0.833 > 0.8$), track chuyển trạng thái từ `initialized`/`tentative` sang `confirmed`.
   - **Giữ confirmed sau miss:** Khi một track đã đạt trạng thái `confirmed` mà bị miss trong FOV LiDAR ở một frame, điểm score bị trừ $-1/\text{window} = -1/6$ (còn $5/6$), nhưng trạng thái vẫn được duy trì là `confirmed` (không bị hạ cấp xuống tentative).
   - **Điều kiện xóa track (`should_delete_track`):** Track bị xóa khỏi danh sách khi thỏa mãn bất kỳ điều kiện nào sau đây (quan hệ OR):
     1. Phương sai vị trí mặt phẳng ngang vượt quá ngưỡng an toàn: $P_{00} > \text{max\_P}$ hoặc $P_{11} > \text{max\_P}$ (với $\text{max\_P} = 3.0^2 = 9.0$).
     2. Track đã `confirmed` nhưng điểm tồn tại giảm xuống dưới ngưỡng xóa: `score < delete_threshold` (ngưỡng 0.6, tương ứng sau 3 frame miss liên tiếp: $1.0 - 3/6 = 0.5 < 0.6$).
     3. Track chưa `confirmed` (`initialized` hoặc `tentative`) có điểm `score <= 0`.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity Assistant (Google DeepMind)
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ cài đặt công thức toán EKF 6D (Part E), mô hình pinhole và FOV camera (Part G), ma trận Mahalanobis và greedy association (Part F), logic vòng đời track (Part H), giải thích các câu hỏi lý thuyết và thiết lập môi trường chạy tích hợp.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy toàn bộ 128 unit tests trong `student/tests` (pass 100%), chạy pipeline tích hợp `fusion-run-lab --fusion compare --seed 0` trên toàn bộ 198 frame Waymo, kiểm tra tính nhất quán giữa `metrics.json` và `grade_run.log`, và chạy script kiểm định `tools/check_submission.py`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
