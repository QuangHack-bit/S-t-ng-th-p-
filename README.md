```
╔═══════════════════════════════════════════════════════════════════════════════════════════════════════════════╗
║                                        ORANGE PI 5B  —  ROS 2 JAZZY                                             ║
║                                                                                                                 ║
║  ┌──────────────────────────────┐   ┌──────────────────────────────┐   ┌──────────────────────────────────┐     ║
║  │ [D] ODOM TÍCH PHÂN + EKF     │──▶│ [A] NAV / SLAM / LiDAR       │──▶│ [B] PUBLISHER /cmd_vel           │     ║
║  │  tích phân twist → pose      │   │  Nav2 · SLAM Toolbox         │   │  Twist: Vx,Vy [m/s], ωz [rad/s]  │     ║
║  │  fusion LiDAR → /odom, /tf   │   │                              │   │  @20–50 Hz                       │     ║
║  └──────────────▲───────────────┘   └──────────────────────────────┘   └────────────────┬─────────────────┘     ║
║                 │                                                                        │                      ║
╚═════════════════╪════════════════════════════════════════════════════════════════════════╪══════════════════════╝
                  │  micro-ROS UART: twist ↑ (50–100 Hz)                                   │  micro-ROS UART: cmd_vel ↓
                  │                                                                        ▼
╔═════════════════╪══════════════════════════════════════════════════════════════════════════════════════════════╗
║                                        STM32F407VGT6                                                            ║
║  ┌──────────────┴───────────────┐                                          ┌──────────────────────────────────┐  ║
║  │ [11] ĐỘNG HỌC THUẬN          │                                          │ [1] micro-ROS NODE               │  ║
║  │  twist: Vx, Vy [m/s], ωz     │                                          │  + WATCHDOG cmd_vel (300 ms)     │  ║
║  └──────────────▲───────────────┘                                          └───────────────┬──────────────────┘  ║
║                 │ ω_filt,i·R_i                                                             ▼                   ║
║                 │                                     ┌──────────────────────────────────┐                     ║
║                 │                                     │ [3] ĐỘNG HỌC NGHỊCH              │                     ║
║                 │                                     │  ma trận 4×3 (d, r, R)           │                     ║
║                 │                                     └───────────────┬──────────────────┘                     ║
║                 │                                                     ▼                                        ║
║                 │                                     ┌──────────────────────────────────┐                     ║
║                 │                                     │ [4] SETPOINT ω₁..ω₄ [rad/s]      │                     ║
║                 │                                     │  + RATE LIMITER |dω/dt| ≤ a_max  │                     ║
║                 │                                     └───────────────┬──────────────────┘                     ║
║                 │                                                     ▼                                        ║
║                 │  ┌───────────────────────┐  ┌───────────────────────┐  ┌───────────────────────┐  ┌──────────┴────────┐  ║
║                 │  │ [10] KALMAN ×4        │  │ [9] QUY ĐỔI VẬN TỐC   │  │ [8] ENCODER A/B       │  │ [5] PI ×4         │  ║
║                 └──┤  scalar, Q,R          │◀─┤  Δcount → rad/s       │◀─┤  quadrature, Timer X  │  │  + ANTI-WINDUP    │  ║
║                    └───────────┬───────────┘  └───────────────────────┘  └───────────▲───────────┘  └──────────────────┘  ║
║                                │ ω_filt,i (hồi tiếp)                                 │                      ▼            ║
║                                └───────────────────────────────────────────────────┐  │        ┌───────────────────────┐   ║
║                                                                                      │  │        │ [6] PWM & DIRECTION   │   ║
║                                                                                      │  │        │  sign(u)→RP/LP        │   ║
║                                                                                      │  │        │  |u| → CCR (0..255)   │   ║
║                                                                                      │  │        └───────────┬───────────┘   ║
║                                                                                      │  │                    ▼             ║
║                                                                                      │  │  ┌─────────────────────────────┐  ║
║                                                                                      │  │  │ [7] PLANT: DRIVER H-bridge  │  ║
║                                                                                      │  └──┤  → MOTOR TorqueNADO 44260   │  ║
║                                                                                         │  │  → BÁNH OMNI (X-Drive)      │  ║
║                                                                                         │  └─────────────────────────────┘  ║
║                                                                                         │   (trục bánh quay → sinh xung A/B)║
╚═════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝
```
# PHÂN TÍCH HỢP ĐỒNG GIAO DIỆN TỪNG KHỐI + GIẢI QUYẾT 4 TRANH CHẤP CỦA BẠN

Trước khi vào chi tiết, tôi chốt một nguyên tắc để bạn đọc sơ đồ không bao giờ nhầm nữa: trong hệ này có **2 loại dữ liệu**:

- **Tín hiệu (signal):** chảy từ khối này sang khối khác, thay đổi mỗi chu kỳ Δt (ví dụ Δcount, ω_raw, u).
- **Tham số (parameter):** nạp một lần từ cấu hình/datasheet, không chảy giữa các khối (d, r, R, q, PPR_OS, N, Δt, Kp, Ki, Kd, Q, R, PWM_MAX).

Mọi “input bắt buộc” dưới đây = **tín hiệu từ khối trước** + **tham số cấu hình**. Thiếu một trong hai, khối đó chạy ra rác.

---

## PHẦN I – HỢP ĐỒNG INPUT / XỬ LÝ / OUTPUT TỪNG KHỐI

### KHỐI [B] – Publisher /cmd_vel (Pi 5)
- **INPUT:** lệnh điều hướng nội bộ từ Nav2 (không phải tín hiệu từ khối khác trong sơ đồ).
- **XỬ LÝ:** đóng gói Twist; publish 20–50 Hz; đặt `linear.z = angular.x = angular.y = 0`.
- **OUTPUT (bắt buộc đủ 3 trường):** `linear.x` [m/s], `linear.y` [m/s], `angular.z` [rad/s] + timestamp gửi.
- **INPUT khối kế [1] cần đúng:** 3 float trên + thời điểm nhận.

### KHỐI [1] – micro-ROS node + WATCHDOG (STM32)
- **INPUT:** 3 float twist [m/s, m/s, rad/s] + timestamp.
- **XỬ LÝ:** (1) deserialize; (2) ghi vào biến chia sẻ với ISR trong critical section; (3) watchdog: `now − t_rx > 300 ms → ép vx=vy=wz=0, valid=0`; (4) lấy twist từ hàng đợi [11] để publish lên.
- **OUTPUT:** struct `{ float vx_cmd [m/s]; float vy_cmd [m/s]; float wz_cmd [rad/s]; uint8 valid; }`.
- **INPUT khối kế [3] cần đúng:** `vx_cmd, vy_cmd, wz_cmd` (đã gate bởi valid).

### KHỐI [3] – ĐỘNG HỌC NGHỊCH
- **INPUT:** `vx_cmd, vy_cmd, wz_cmd` + tham số `d = 0.7071`, `r = 0.5(wheelbase+track_width)`, `R1..R4 [m]`.
- **XỬ LÝ:** áp đúng 4 công thức file:
  ω1 = ( d·Vx + d·Vy + r·ωz )/R1 ; ω2 = (−d·Vx + d·Vy + r·ωz)/R2 ; ω3 = (−d·Vx − d·Vy + r·ωz)/R3 ; ω4 = ( d·Vx − d·Vy + r·ωz)/R4.
- **OUTPUT:** mảng `omega_raw_set[4]` **[rad/s]**, có dấu, thứ tự FR, FL, RL, RR.
- **INPUT khối kế [4] cần đúng:** chính mảng 4 phần tử [rad/s] này.

### KHỐI [4] – BUFFER SETPOINT + RATE LIMITER + CHẾ ĐỘ TEST
- **INPUT:** `omega_raw_set[4]` [rad/s] từ [3]; tham số `a_max [rad/s²]`, `Δt`; lệnh chế độ test (UART/debug).
- **XỬ LÝ:** xem chi tiết Phần II dưới.
- **OUTPUT:** mảng `v_set[4]` **[rad/s]** — đây chính là biến `v_set` trong đoạn code PID của file (`float error = v_set - v_act;`).
- **INPUT khối kế [5] cần đúng:** `v_set[4]` [rad/s] khớp thứ tự bánh với `omega_filt[4]`.

### KHỐI [5] – PI ×4 + ANTI-WINDUP
- **INPUT:** `v_set[4]` [rad/s] từ [4]; `omega_filt[4]` [rad/s] từ [10]; tham số Kp, Ki, Kd, dt; trạng thái nội bộ `integral[4]`, `prev_error[4]`.
- **XỬ LÝ:** với từng bánh i: `e = v_set − v_act`; `p = Kp·e`; `d = Kd(e−e_prev)/dt`; `tentative = p + (integral + Ki·e·dt) + d`; chỉ cộng integral nếu `PWM_MIN < tentative < PWM_MAX` với **PWM_MIN = −255, PWM_MAX = +255**; `u = p + integral + d`; clamp ±255.
- **OUTPUT:** mảng `u[4]` **PWM có dấu, đơn vị count, miền [−255, +255]**.
- **INPUT khối kế [6] cần đúng:** `u[4]` signed count.

### KHỐI [6] – PWM & DIRECTION
- **INPUT:** `u[4]` signed count.
- **XỬ LÝ:** với từng bánh: `if u>0 → RP=1, LP=0; else RP=0, LP=1` (đúng logic file); `pwm_mag = |u|`; ghi CCR timer (nếu ARR ≠ 255 thì scale `pwm_mag/255·ARR`); chế độ manual/open-loop mới dùng `PWM = |W|·C, C = 255/ωmax`.
- **OUTPUT:** `pwm_mag[4]` uint8 0..255 (thanh ghi CCR) + `rp[4], lp[4]` (mức GPIO) — **ghi ra phần cứng trước driver**.
- **INPUT khối kế [7]:** tín hiệu điện PWM + 2 chân chiều từng kênh.

### KHỐI [7] – PLANT (Driver → TorqueNADO 44260 → bánh Omni)
- **OUTPUT vật lý:** góc và tốc độ trục bánh ω_wheel(t) [rad/s] + quay cơ khí truyền sang encoder.

### KHỐI [8] – ENCODER QUADRATURE
- **INPUT:** xung A/B; counter timer encoder; capture Timer X.
- **XỬ LÝ:** giải mã q = 4; counter có dấu; chiều theo chuỗi `00→10→11→01→00`; phát hiện transition bất hợp lệ; mỗi ngắt Timer Y: `Δcount = Current_Count − Previous_Count`; ghi `t_first, t_last`.
- **OUTPUT:** `dcount[4]` (int32, **số cạnh q=4 trong cửa sổ, có dấu**), `dir_flag[4]`, `t_first[4], t_last[4]` [tick µs].
- **INPUT khối kế [9] cần đúng:** `dcount[4]`, `t_first/t_last`, tham số q, PPR_OS, Δt.

### KHỐI [9] – QUY ĐỔI VẬN TỐC
- **INPUT:** `dcount[4]`, `t_first/t_last` + tham số q, PPR_OS, Δt.
- **XỬ LÝ:** xem Phần III (chứng minh công thức).
- **OUTPUT:** `omega_raw[4]` **[rad/s] có dấu** + `rpm[4]` (chỉ để log, = ω·9.5493).
- **INPUT khối kế [10] cần đúng:** `omega_raw[4]` đóng vai trò phép đo z.

### KHỐI [10] – KALMAN ×4
- **INPUT:** z = `omega_raw[4]` [rad/s]; tham số Q ∈ [0.01, 0.5], R ∈ [1.0, 10.0]; trạng thái x̂, P từng bánh.
- **XỬ LÝ:** predict `P⁻ = P + Q`; update `K = P⁻/(P⁻+R)`, `x̂ = x⁻ + K(z−x̂⁻)`, `P = (1−K)P⁻`.
- **OUTPUT:** `omega_filt[4]` **[rad/s]** — đi **hai nơi**: [5] (v_act) và [11].
- **INPUT khối kế cần đúng:** [5] nhận `omega_filt[i]` khớp bánh i; [11] nhận cả 4.

### KHỐI [11] – ĐỘNG HỌC THUẬN
- **INPUT:** `omega_filt[4]` [rad/s] + tham số R1..R4, d, r.
- **XỬ LÝ:** 3 công thức tường minh file: Vx = 0.25(ω1R1 − ω2R2 − ω3R3 + ω4R4)/d ; Vy = 0.25(ω1R1 + ω2R2 − ω3R3 − ω4R4)/d ; ωz = 0.25(ΣωiRi)/r.
- **OUTPUT:** `vx_act, vy_act [m/s]`, `wz_act [rad/s]` + timestamp → hàng đợi publish của [1].
- **INPUT khối kế [D] cần đúng:** bộ ba twist + dt giữa 2 gói.

### KHỐI [D] – TÍCH PHÂN ODOM + EKF (Pi 5)
- **INPUT:** twist [m/s, m/s, rad/s] + dt; pose LiDAR/SLAM.
- **XỬ LÝ:** θₖ = θₖ₋₁ + ωz·Δt; xₖ = xₖ₋₁ + (Vx·cosθ − Vy·sinθ)·Δt; yₖ = yₖ₋₁ + (Vx·sinθ + Vy·cosθ)·Δt; fusion EKF.
- **OUTPUT:** pose x, y [m], θ [rad] + covariance → `/odom`, `/tf`.
- **INPUT khối kế [A]:** pose đã fusion.

---

## PHẦN II – KHỐI [4]: “ω_set,i Ở ĐÂU RA?”

Bạn hỏi đúng chỗ then chốt. Trả lời thẳng: **ω_set,i KHÔNG tự sinh ra ở khối [4]. Nó CHÍNH LÀ đầu ra của khối [3].** Khối [4] chỉ là trạm “tinh chỉnh và giữ” bộ số đó trước khi đưa vào PI.

Chuỗi nhân quả:

1. Khối [3] nhận [Vx, Vy, ωz] và sinh ra 4 con số ω1..ω4 [rad/s].
2. Bốn con số đó, xét từ góc nhìn của vòng PI, chính là **giá trị đặt mong muốn** cho từng bánh → ta đổi tên gọi thành ω_set,1..4 (set = setpoint).
3. Khối [4] nhận 4 con số đó và làm 3 việc:
   - **Buffer/giữ:** giữa hai gói cmd_vel liên tiếp, PI vẫn cần v_set mỗi chu kỳ Δt; [4] giữ giá trị mới nhất.
   - **Rate limiter:** giới hạn tốc độ thay đổi setpoint: `v_set(k) = clamp( ω_set(k), v_set(k−1) ± a_max·Δt )`.
   - **Chế độ test:** cho phép ghi đè setpoint thủ công (ví dụ đặt cả 4 bánh = 10.47 rad/s để làm đúng nhiệm vụ số 2 trong file: “so sánh với 10.47 rad/s”).
4. Đầu ra [4] chính là biến `v_set` trong code PID của file. Không có [4], biến `v_set` trong `float error = v_set - v_act;` không có nguồn cấp.

**Ví dụ số để bạn thấy rõ:** Vx = 0.2 m/s, Vy = 0, ωz = 0; d = 0.7071, r = 0.4, R = 0.05.
- [3] xuất: ω_set = [+2.828, −2.828, −2.828, +2.828] rad/s.
- [4] với a_max = 20 rad/s², Δt = 0.01 s → mỗi chu kỳ v_set chỉ được tăng tối đa 0.2 rad/s. Chuỗi v_set bánh 1: 0 → 0.2 → 0.4 → … → 2.8 → 2.828 (khoảng 15 chu kỳ = 150 ms).
- [5] nhận v_set đã “mềm” đó, sai số e không bao giờ nhảy dựng → không kích hoạt windup, không trượt bánh Omni.

Nếu bỏ [4]: cmd_vel bước nhảy tạo e lớn tức thời → integral tích lũy mạnh ngay cả khi có anti-windup (vì anti-windup chỉ chặn khi chạm trần), bánh trượt, odom hỏng. Đó là lý do [4] tồn tại.

---

## PHẦN III – KHỐI [9]: “2π·pulse A” HAY “2π·Δcount”? AI SAI?

**Cả hai đều không sai, nhưng phát biểu của bạn đang thiếu hai phép chia, nên nếu code đúng chữ “2π·pulse A” thì bạn sai đơn vị.** Tôi chứng minh bằng phân tích thứ nguyên, dùng đúng định nghĩa trong file của bạn.

**Bước 1 – Một xung mang bao nhiêu góc?** Encoder có PPR xung/kênh/vòng. Vậy 1 xung kênh A tương ứng góc quay:

Δθ_per_pulse = 2π / PPR_OS [rad/xung] (với giải mã q lần: 2π/(q·PPR_OS)).

Do đó “2π·pulseA” **chưa phải góc quay**; muốn ra góc phải chia PPR: `góc = 2π·(số xung)/(q·PPR_OS)`.

**Bước 2 – Muốn ra rad/s phải chia thời gian.** Góc đó tích lũy trong cửa sổ Δt của Timer Y. Vậy:

ω = 2π·(số xung trong cửa sổ) / (q·PPR_OS·Δt).

**Bước 3 – “Số xung kênh A trong cửa sổ” tên là gì?** Đúng định nghĩa file: `Δcount = Current_Count − Previous_Count`. Ở chế độ 1x, Δcount chính là số xung A giữa hai ngắt Timer Y. **Vậy “pulse A” của bạn và Δcount của tôi là MỘT, khi bạn hiểu “pulse A” là số xung đếm được trong cửa sổ.** Còn nếu bạn hiểu “pulse A” là một xung đơn lẻ, thì 2π·1 xung = 2π rad — vô nghĩa về vận tốc.

**Kiểm chứng bằng mốc 10.47 rad/s của chính file bạn:** PPR_OS = 1024 (1x), Δt = 0.01 s, bánh quay 100 rpm = 10.47 rad/s → số vòng trong cửa sổ = 100/60·0.01 = 0.016667 vòng → số xung A = 0.016667·1024 ≈ 17.07 = Δcount. Thế vào công thức của tôi: ω = 2π·17.07/(1·1024·0.01) = 10.47 rad/s ✔. Thế vào “2π·pulseA” nguyên văn của bạn: 2π·17.07 ≈ 107.25 — không phải rad/s, đó là góc của 17 vòng. Sai đơn vị rõ ràng.

**Ba dạng tương đương, chọn dạng nào cũng ra một kết quả:**
- Dạng đếm (file dùng, Timer Y): ω = 2π·Δcount/(q·PPR_OS·Δt).
- Dạng tần số: ω = 2π·f_A/(q·PPR_OS), với f_A = Δcount/Δt [Hz].
- Dạng chu kỳ (Timer X, cho tốc độ thấp): với m cạnh được ghi timestamp từ t_first đến t_last: ω = 2π·(m−1)/(q·PPR_OS·(t_last − t_first)). Dùng khi Δcount < ~4 để khỏi lượng tử hóa thô.

Kết luận: tôi không sai; công thức của bạn **đúng ý nhưng thiếu /PPR và /Δt**; khi bổ sung, nó trùng khít công thức tôi đưa. Và khối [9] bắt buộc phải nhận Δcount (hoặc m + t_first/t_last), không thể nhận “một pulse”.

---

## PHẦN IV – “ENCODER NẰM LUÔN TRONG ĐỘNG CƠ” THÌ LÀ TRƯỚC HAY SAU HỘP GIẢM TỐC?

**“Nằm trong động cơ” KHÔNG trả lời được câu hỏi trước/sau hộp giảm tốc.** Trong gearmotor tích hợp encoder, đĩa encoder thường gắn trên **trục rotor motor (TRƯỚC giảm tốc)** vì rẻ và cho phân giải cao; một số ít gắn trục ra. Với TorqueNADO 44260 bạn phải chốt bằng một trong hai cách:

**Cách 1 – Đọc datasheet:** tìm xem thông số encoder ghi “per motor shaft revolution” hay “per output shaft revolution”, và gear ratio N. Nếu ghi theo trục motor → PPR_OS = PPR·N.

**Cách 2 – Test 1 vòng (chắc chắn 100%, không cần datasheet):** đánh dấu trục RA; quay tay đúng 1 vòng trục ra; đếm xung kênh A (1x). Số đếm được = PPR_OS trực tiếp. So sánh: nếu ≈ PPR datasheet → encoder SAU giảm tốc; nếu ≈ PPR·N → encoder TRƯỚC giảm tốc.

**Hậu quả nếu nhầm:** giả sử N = 39 mà bạn lấy PPR motor làm PPR_OS, vận tốc báo ra sẽ **sai 39 lần**: đặt 10.47 rad/s, PI thấy “đã đạt 408 rad/s” → cắt PWM ngay → bánh không bao giờ quay đúng; odom bay vô lý. Đây là lỗi tôi gặp nhiều nhất ở sinh viên làm AMR gearmotor.

**Công thức thống nhất ở khối [9] chỉ có MỘT, mọi trường hợp:**

ω_wheel = 2π·Δcount / (q·PPR_OS·Δt), với PPR_OS = PPR·N (encoder trước giảm tốc) hoặc PPR_OS = PPR_output (encoder sau giảm tốc).

Nghĩa là khái niệm “encoder trước hộp giảm tốc” mà tôi nêu chỉ là cách nói của trường hợp PPR_OS = PPR·N; bạn không cần hai công thức riêng, bạn chỉ cần **một giá trị PPR_OS đúng**, lấy từ test 1 vòng.

---

## PHẦN V – QUY TRÌNH MATLAB TUNING PI “THẬT CHÍNH XÁC” THEO ĐÚNG MỤC 4.2.3 CỦA FILE

**Bước 1 – Thu dữ liệu bước nhảy (vòng hở):** nâng bánh khỏi sàn; PWM nhảy 0 → U₀ = 127; ghi ω_wheel mỗi 1 ms trong ~2–3 s (dùng đầu ra khối [9], chưa lọc hoặc lọc nhẹ).

**Bước 2 – Đo K và τ đúng file:** K = ω_ss/U₀ với ω_ss = trung bình ~50 mẫu cuối; τ = thời điểm đầu tiên ω đạt 63.2%·ω_ss tính từ lúc nhảy bậc. Đơn vị K: (rad/s)/PWM-count.

**Bước 3 – MATLAB:**

```matlab
Ts = 0.001;
u  = U0 * ones(N,1);          % chuoi PWM nhay bac
y  = omega_log(:);            % rad/s
data = iddata(y, u, Ts);
sys  = tfest(data, 1, 0, 0);  % mo hinh bac 1: K/(tau*s+1)
% Kiem tra chéo với K, tau đo tay ở Bước 2

% Cach A: pidtune chính xác, biên pha 60 deg, bandwidth 10-15 Hz
opts = pidTuneOptions('PhaseMargin',60,'Bandwidth',2*pi*12);
C    = pidtune(sys,'PI',opts);

% Cach B: giải tích IMC (chọn lambda = hằng số thời gian vòng kín mong muốn)
% lambda vd 0.05 s:  Kp = tau/(K*lambda);  Ki = 1/(K*lambda)

% Roi rac hoa dung dt = 0.01 s, dang Tustin khop code tich phan cua file
Cd = c2d(C, 0.01, 'tustin');
[Kp, Ki] = piddata(Cd);       % dang parallel: u = Kp*e + Ki*int(e)
```

**Bước 4 – Kiểm tra thứ nguyên gain:** vì K có đơn vị (rad/s)/count, nên Kp = τ/(K·λ) có đơn vị count/(rad/s) và Ki có đơn vị count/rad — khớp đúng code file `integral += Ki*error*dt` với error [rad/s], dt [s], integral [count].

**Bước 5 – Nghiệm thu trên phần cứng:** step v_set = 10.47 rad/s (mốc của file): yêu cầu rise time ≈ 3·λ, overshoot < 5–10%, sai số xác lập → 0 (đúng vai trò Ki trong file); sau đó test bão hòa: step v_set = 2·ωmax để kiểm tra anti-windup xả tích phân nhanh khi hạ lệnh. Tune trên 1 bánh đại diện, áp dụng chung 4 bánh, rồi tinh chỉnh từng bánh nếu lệch > 5%.

---

## TÓM TẮT 4 ĐIỂM BẠN VỪA HỎI

1. **Khối [4]:** ω_set,i là đầu ra khối [3] được đổi tên; [4] chỉ buffer + rate-limit + chế độ test; đầu ra [4] chính là `v_set` trong code PID của file.
2. **Khối [9]:** “2π·pulseA” của bạn thiếu /PPR và /Δt; bổ sung xong thì trùng công thức Δcount của tôi; Δcount chính là số xung A trong cửa sổ (1x).
3. **Encoder trong motor:** phải chốt trước/sau giảm tốc bằng test 1 vòng; mọi trường hợp gom về một công thức duy nhất qua PPR_OS.
4. **PI bằng MATLAB:** theo đúng 4.2.3 của file (đo K, τ) → tfest/pidtune hoặc IMC → c2d Tustin dt = 0.01 s → nghiệm thu tại mốc 10.47 rad/s.

Bạn gửi tôi kết quả test 1 vòng (số xung/vòng trục ra) và giá trị K, τ đo được, tôi sẽ tính sẵn bộ Kp, Ki chuẩn đơn vị count/(rad/s) để nạp thẳng vào khối [5].
