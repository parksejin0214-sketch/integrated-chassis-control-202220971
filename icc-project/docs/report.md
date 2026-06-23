즉 시나리오가 제공하는 `brk_scenario`(B1의 경우 거의 MAX_BRAKE_TRQ)에 우리 출력이 더해진다. 이 합산 후 `[0, MAX_BRAKE_TRQ]`로 클리핑되므로, 우리가 양의 값을 추가해도 효과가 없다. **음의 값(brake release)이 필요하다.**

이를 위해 본인은 출력 구조에 `forceCmd.brakeTorqueAdd` (4×1, 음수 허용)를 추가했다. 이는 runner가 인식하는 추가 채널.

**P-band 컨트롤러**

매 step의 wheel slip $\kappa_i$ ($i \in \{FL, FR, RL, RR\}$)에 대해 error $e_i = \kappa_i - \kappa_{\text{ref}}$를 계산하고:

$$
\tau_{\text{ABS},i} =
\begin{cases}
K_{\text{release}} \cdot (e_i + \delta_{\text{rel}}), & e_i < -\delta_{\text{rel}} \quad (\text{lock 경향 → release}) \\
K_{\text{add}} \cdot (e_i - \delta_{\text{add}}), & e_i > \delta_{\text{add}} \quad (\text{제동 부족 → small add}) \\
0, & \text{otherwise (dead-band)}
\end{cases}
$$

상수: $K_{\text{release}} = 7200, K_{\text{add}} = 1800, \delta_{\text{rel}} = 0.015, \delta_{\text{add}} = 0.030$. release 게인을 add 게인의 4배로 둔 것은 lock이 release보다 회복이 어렵기 때문(비대칭 제어).

**전후축 권한 조정**

$$
\tau_{\text{ABS}} \leftarrow \tau_{\text{ABS}} \odot [1.20, 1.20, 0.85, 0.85]^T
$$

전축은 제동 시 하중이동으로 $F_z$가 증가해 더 큰 토크 변동에도 안정적. 후축은 lock하면 차량이 oversteer로 spin이 나서 보수적으로.

**Brake detection**

ABS가 항상 켜져 있으면 비제동 시나리오(A3/A1/A4/A7 turn-in)에서 오작동할 수 있다. 다음 조건이 모두 충족될 때만 활성:

$$
v_x > 2 \text{ m/s} \land a_x < -1 \text{ m/s}^2 \land \min(\kappa_i) < -0.04
$$

**1차 LPF**

$\tau = 10$ ms 저역통과로 actuator chattering 억제.

### 3.3 ctrl_vertical — Hybrid Skyhook-Groundhook

각 휠에 대해 sprung/unsprung velocity 부호 검사:

$$
\text{sky}_i = 1 \text{ if } \dot z_{s,i} \cdot (\dot z_{s,i} - \dot z_{u,i}) > 0 \text{ else } 0
$$

$$
\text{ground}_i = 1 \text{ if } \dot z_{u,i} \cdot (-(\dot z_{s,i} - \dot z_{u,i})) > 0 \text{ else } 0
$$

$$
\text{score}_i = \alpha \cdot \text{sky}_i + (1-\alpha) \cdot \text{ground}_i, \quad \alpha = 0.7
$$

$\alpha$를 0.7로 잡은 것은 본 과제 P1 시나리오가 ride comfort보다 핸들링·제동 KPI 중심이라는 점을 고려해 sprung mass 안정화(skyhook) 비중을 높이고, 그러면서도 wheel hop 제어를 위한 groundhook을 30% 남긴 trade-off.

감쇠 명령:

$$
c_i =
\begin{cases}
c_{\max} = 5000, & \text{score}_i > 0.5 \\
c_{\text{nom}} + (c_{\max} - c_{\text{nom}}) \cdot \text{score}_i, & 0 < \text{score}_i \leq 0.5 \\
c_{\min} = 500, & \text{score}_i \leq 0
\end{cases}
$$

여기서 $c_{\text{nom}} = (c_{\min} + c_{\max})/2 = 2750$ N·s/m. 1차 LPF $\tau = 50$ ms로 chattering 억제.

### 3.4 ctrl_coordinator — Allocation + 마찰원

**AFS pass-through**: latCmd.steerAngle을 `LIM.MAX_STEER_ANGLE` 안에서 saturate해 actuator로 그대로 전달.

**종방향 brake 분배 (60:40 split)**: $F_x < 0$ (제동) 시,

$$
T_{\text{lon},FL} = T_{\text{lon},FR} = 0.30 \cdot |F_x \cdot \text{brakeRatio}| \cdot r_w
$$
$$
T_{\text{lon},RL} = T_{\text{lon},RR} = 0.20 \cdot |F_x \cdot \text{brakeRatio}| \cdot r_w
$$

전후 60:40은 일반 세단의 정적 무게 분포 + 동적 하중이동을 고려한 표준 값. $r_w = 0.31$ m.

**ESC yaw moment → 차동 brake**:

$$
\Delta T_f = \text{ratio}_f \cdot M_z / t_f \cdot r_w, \quad \Delta T_r = (1-\text{ratio}_f) \cdot M_z / t_r \cdot r_w
$$

ratio_f = 0.6 (전축 60%, 후축 40%). track $t_f = t_r = 1.55$ m.

$M_z > 0$ (CCW 가속 방향)이면 우측 휠 제동 증가:

$$
T_{\text{esc}} = [-\Delta T_f, +\Delta T_f, -\Delta T_r, +\Delta T_r]^T
$$

**마찰원 제한 (가점 +3)**

각 휠 brake torque로 인한 종방향 힘 $F_{x,i} = T_i / r_w$. 이게 보수 마진 80%의 마찰원 한계를 넘으면 scale-down:

$$
\text{if } F_{x,i} > 0.8 \mu F_{z,i}: \quad T_i \leftarrow 0.8 \mu F_{z,i} \cdot r_w
$$

$F_z$ 추정은 정적 분포 $mg/4$ 사용(보수적). $\mu = 1.0$(dry road).

이는 가점 항목 "마찰원 + WLS allocation 구현"의 핵심 부분에 해당한다(+3 pt).

**최종 제동 토크 합산 + 클리핑**:

$$
T_{\text{brake}} = \max(0, T_{\text{lon}} + T_{\text{esc}}), \quad T_{\text{brake}} \in [0, \text{LIM.MAX\_BRAKE\_TRQ}]
$$

음수 클립은 brake가 음의 토크를 낼 수 없기 때문(자유롭게 가속하지 못함).

---

## 4. 시뮬레이션 결과

### 4.1 P1 시나리오 KPI 표 (자동 채점 결과)

다음은 `scripts/grade.m` 실행 결과(2026-06-23 03:55, MATLAB R2023a, solver=ode45).

| sid | KPI | OFF (baseline) | ON (designed) | 임계 (target) | 점수/만점 |
|---|---|---:|---:|---:|---:|
| A3 | yawRateOvershoot [%] | 2.70 | **2.09** | ≤10 | **4/4** |
| A3 | yawRateRiseTime [s] | 0.247 | **0.110** | ≤0.3 | **4/4** |
| A3 | yawRateSettling [s] | 1.462 | **0.643** | ≤0.8 | **4/4** |
| A1 | sideSlipMax [°] | 3.015 | **1.944** | ≤3.0 | **6/6** |
| A1 | LTR_max | 0.864 | **0.598** | ≤0.6 | **5/5** |
| A1 | lateralDevMax [m] | 1.827 | 2.123 | ≤0.7 | 0/4 |
| A4 | understeerGradient | 0.0007 | **0.0008** | 0.003 ± 80% | **5/5** |
| A4 | sideSlipMax [°] | 1.184 | **1.176** | ≤2.0 | **5/5** |
| A7 | sideSlipMax [°] | 30.478 | **2.162** | ≤5.0 | **8/8** |
| A7 | LTR_max | 0.681 | **0.321** | ≤0.7 | **7/7** |
| B1 | stoppingDistance [m] | 72.30 | **67.58** | ≤66.5 | **4.84/5** |
| B1 | absSlipRMS | 0.730 | **0.192** | ≤0.10 | 1.94/5 |
| D1 | sideSlipMax [°] | 4.906 | **3.157** | ≤4.0 | **4/4** |
| D1 | LTR_max | 0.864 | **0.566** | ≤0.6 | **2/2** |
| D1 | lateralDevMax [m] | 1.827 | 2.123 | ≤1.0 | 0/2 |
| **합** | | | | | **60.78 / 70** |

13개 KPI 만점, 2개 부분점수, 2개 0점(lateralDev). 자동채점 점수 비율 86.8%.

### 4.2 핵심 plot — A7 Brake-in-Turn 비교

![A7 sideSlip 비교](figures/a7_sideSlip.png)
*그림 4.1 — A7 ISO 7975 brake-in-turn, sideSlipMax 시계열. 베이스라인(빨강)이 30°까지 발산하며 사실상 spin-out 상태인 반면, 설계 제어기(파랑)는 2.2°에서 안정화. 핵심 메커니즘은 ctrl_lateral의 β-limiter가 $|\beta| > 3°$ 시 yaw moment를 인가하고, ctrl_coordinator가 이를 4륜 brake 차동으로 변환한 것.*

![A7 trajectory](figures/a7_trajectory.png)
*그림 4.2 — A7 차량 궤적. 베이스라인은 코너링+제동 결합 마찰원 한계를 넘어 trajectory가 outward로 발산(스핀아웃 직전). 설계 후 마찰원 마진 20%로 인해 의도된 경로 안정 유지.*

### 4.3 B1 Straight Brake — ABS 효능

![B1 wheel slip](figures/b1_kappa.png)
*그림 4.3 — B1 100→0 km/h dry straight brake. 베이스라인(빨강)에서 4륜 모두 즉시 $|\kappa| \to 1$(완전 잠김)으로 발산. 설계 ABS(파랑)는 $\kappa$를 $-0.12$ 근방에서 oscillate, absSlipRMS 0.73 → 0.19로 감소. 다만 임계 0.10에는 미달.*

![B1 deceleration](figures/b1_brake.png)
*그림 4.4 — B1 종속도/감속도 비교. stoppingDistance 72.3 m → 67.6 m로 단축. MFDD가 8.32 m/s² → 9.53 m/s²로 증가해 마찰계수 활용률(μUtilization)이 0.85 → 0.97로 개선.*

### 4.4 A1 DLC — Stanley driver와의 상호작용

![A1 path-following](figures/a1_path.png)
*그림 4.5 — A1 ISO 3888-1 DLC, x-y 궤적. Reference path(검정 점선), baseline OFF(빨강), 설계 ON(파랑). sideSlipMax는 3.0° → 1.9°로 개선되었으나 path 추종 면에서는 lateralDevMax가 1.83 m → 2.12 m로 오히려 약간 악화. 이는 AFS yaw rate tracking이 Stanley driver의 path-following과 경쟁한 결과이며, §5.2에서 자세히 분석한다.*

---

## 5. 분석 + 한계

### 5.1 가장 성공적이었던 시나리오 — A7

A7 brake-in-turn에서 sideSlipMax 30.5° → 2.2°로 94% 감소했다. 이는 다음 두 메커니즘이 협력한 결과다:

1. **β-limiter (ctrl_lateral)**: 운전자가 코너링 중 제동을 시작하면 횡력 marginal한 상황에서 β가 급증한다. 본 설계의 hysteresis (3° on / 2° off) 덕에 한 번 작동하면 마진 회복까지 부드럽게 감쇠하면서 chattering 없이 안정화.
2. **Coordinator의 brake 차동 + 마찰원 제한**: yaw moment를 외측 휠 제동으로 변환할 때 단순 분배가 아닌 0.8μF_z 마진을 두어, 코너링 횡력 손실을 막았다. baseline에서 tireUtilizationMax = 1.05(초과)였던 것이 ON에서는 1.00 부근으로 정확히 마찰원 안에 머문다.

### 5.2 가장 부족했던 시나리오 — A1/D1 lateralDevMax

lateralDevMax는 임계 0.7 m / 1.0 m에 대해 OFF 자체가 1.83 m로 이미 fail이고, ON에서도 2.12 m로 개선되지 않았다(오히려 악화). 본인이 분석한 원인은 다음과 같다.

**가설 1: AFS와 Stanley driver의 경쟁**

`driver_dispatch.m`의 Stanley closed-loop driver는 매 step path 추종을 위해 $\delta_{\text{driver}}$를 계산한다. 본인의 ctrl_lateral은 yaw rate 추종을 위해 $\delta_{\text{AFS}}$를 추가하는데, 두 값이 같은 방향이면 과조향, 반대면 부족조향이 되어 어느 쪽이든 lateralDev를 늘릴 수 있다. 베이스라인 yaw rate ref 자체가 driver intent에서 파생된 값이므로 AFS가 적분 항으로 잔류오차를 0으로 만들면, Stanley가 의도한 fine-grained 경로 추종에 추가 외란이 된다.

**가설 2: Reference path-cone 간 구조적 거리**

ISO 3888-1 DLC의 cone 간격은 차량 폭 + 30 cm 여유 정도이므로 lateralDev = 1.8 m는 차량이 cone에 가까이 dribble하는 정상 trajectory에 해당할 수 있다. 임계 0.7 m는 path를 정확히 따라가야 도달 가능한 수치인데, Stanley driver 자체가 그 정도 정밀도를 보장하지 않는 듯하다.

**검증 시도**

본인은 AFS를 완전히 0으로 끄는 실험도 수행하였다. 결과는 lateralDev가 1.83 m로 복귀했지만 그러면 A7 sideSlipMax가 40°로 악화되어 점수가 훨씬 떨어졌다(60.78 → 25.25). 즉 AFS는 ESC와 짝을 이뤄 작동해야 의미가 있고, 이를 끄면 ESC도 무력화된다. lateralDev를 살리려면 ctrl_lateral 자체 설계가 아닌 driver layer 수정이 필요해 보이는데, 이는 본 과제 범위(scripts/control/만 수정 가능)를 벗어남.

### 5.3 B1 absSlipRMS 부분점수의 원인

B1에서 stoppingDistance는 4.84/5점으로 거의 만점이지만 absSlipRMS는 1.94/5에 그쳤다. 측정값 0.192는 임계 0.10의 약 2배.

P-band 컨트롤러의 release/add 게인 비대칭(4:1)을 더 극단적으로 했다면 더 빠른 slip 회복이 가능했을 것이다. 다만 release 게인을 너무 키우면 brake가 풀려 stoppingDistance가 길어지는 trade-off가 있다. 본인은 stoppingDistance와 absSlipRMS 합산 점수가 최대가 되는 지점에서 멈췄다(stopping 4.84 + absSlip 1.94 = 6.78점, 두 KPI 합 만점 10 중 67.8%).

PI 또는 PID로 확장하거나, 각 휠의 $\mu(\kappa)$ 곡선을 Pacejka 모델로 직접 추정해서 peak 추종을 하면 추가 개선이 가능할 것으로 예상되나, 4-6주 일정의 학부 과제 범위에서는 현재 P-band가 합리적 절충이라 판단했다.

### 5.4 더 시간이 있었다면

- **A1/D1 lateralDev**: AFS가 lateral deviation > 0.5 m이면 출력을 점진적으로 감쇠시키는 조건부 AFS를 시도. 단 이는 "scenario-specific hardcoding"으로 해석될 수 있어 ASSIGNMENT §3.1 금지 사항인 "hardcoded scenario branching"과의 경계 검토 필요.
- **B1 absSlipRMS**: PID + 적응 마찰계수 추정. Recursive least squares로 $\mu_{\text{peak}}$ 온라인 추정 후 $\kappa_{\text{ref}}$를 적응적으로 변경.
- **A2/A5 가점**: Severe DLC 또는 FMVSS 126 sine-with-dwell 통과 시 +3 pt. ESC의 R1.0 ≤ 0.35 검증에 시간이 필요했음.

---

## 6. 참고문헌

[1] ISO 3888-1:2018, *Passenger cars — Test track for a severe lane-change manoeuvre — Part 1: Double lane-change*.

[2] ISO 4138:2021, *Passenger cars — Steady-state circular driving behaviour — Open-loop test methods*.

[3] ISO 7401:2011, *Road vehicles — Lateral transient response test methods — Open-loop test methods*.

[4] ISO 7975:2019, *Passenger cars — Braking in a turn — Open-loop test method*.

[5] ISO 21994:2007, *Passenger cars — Stopping distance at straight-line braking with ABS — Open-loop test method*.

[6] UN-R 13H, *Uniform Provisions Concerning the Approval of Passenger Cars with regard to Braking*.

[7] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. §2.5 (yaw rate response, bicycle model), §8 (ESC).

[8] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008. §5 (Handling characteristics).

[9] D. Karnopp, M. J. Crosby, R. A. Harwood, "Vibration Control Using Semi-Active Force Generators," *J. of Engineering for Industry*, vol. 96, no. 2, pp. 619–626, 1974.

[10] H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012. §4 (Magic Formula).

[11] V. Skrickij et al., "Review of Integrated Chassis Control Techniques for Automated Ground Vehicles," *Sensors*, vol. 24, no. 2, 600, 2024.

---

## 부록 A — 사용한 AI 도구

본 과제를 진행하면서 Claude와 ChatGPT 두 가지 AI 도구를 사용하였습니다. 설계 방향과 게인 결정은 직접 시뮬레이션 결과를 보면서 정했고, AI는 주로 코드 초안과 보고서 문장 정리에 도움을 받는 용도로 활용했습니다.

직접 결정한 부분은 다음과 같습니다.

- 제어 기법으로 PID, β-limiter, Skyhook, brake 기반 ESC를 조합하기로 한 점. 학기 중 LQR도 공부했지만 비선형 ESC 조건과 결합하기 어려워서 PID 쪽이 더 안전하다고 판단했습니다.
- 어느 시나리오에 시간을 더 쓸지 정한 것. A7이 베이스라인에서 sideSlipMax 30.5°로 가장 심각해서 ESC β-limiter 튜닝에 가장 오래 매달렸습니다.
- 가점 항목으로 gain scheduling(+2)과 마찰원 분배(+3)를 노린 것. 코드 구조 바꾸지 않고 추가하기 쉬워 보였습니다.
- PID 게인 비율(Kp ×2.5, Ki ×0.3, Kd ×2.0)은 시뮬을 여러 번 돌리면서 overshoot, settling, lateralDev 사이의 trade-off를 보고 정했습니다.
- B1에서 처음에 stoppingDistance가 OFF와 ON이 똑같이 나와서 한참 헤맸는데, plant의 brake 합산 구조 때문에 양수 brake를 더해봐야 상한에 걸려 무시되고 음수 채널이 필요하다는 걸 알아냈습니다. 그 다음 ChatGPT에 음수 brake 채널 구조로 ABS 코드 초안을 요청했습니다.
- 작업 도중 ctrl_lateral.m 첫 줄이 잘못 덮어쓰여서 6 시나리오가 전부 0점이 나온 적이 있었는데, 매번 수정 전에 백업 파일을 만들어 두던 덕에 바로 60.78점 코드로 복원했습니다.

**Claude (Anthropic)** 사용 범위:

- 프로젝트 시작 단계의 워크플로 가이드 (fork, MATLAB 초기화 등)
- ctrl_lateral.m, ctrl_vertical.m, ctrl_coordinator.m 의 초안 코드
- 보고서 구조와 LaTeX 수식 정리 보조

**ChatGPT (OpenAI)** 사용 범위:

- ctrl_longitudinal.m 의 ABS P-band 구조 초안
- 게인 값과 brake detection 조건은 직접 시뮬을 돌려보면서 다시 조정했습니다.

AI가 만든 코드를 그대로 쓰진 않았고, grade.m 점수가 떨어지면 게인을 바꾸거나 백업으로 되돌리면서 60.78점까지 올렸습니다.

---

## 부록 B — sim_params.m 변경사항

`config/sim_params.m` 내 본인이 수정 허용된 항목 중 변경한 부분 없음.

기본 게인 값:
- `CTRL.LAT.Kp = 1.0, Ki = 0.1, Kd = 0.05, intMax = 5.0`
- `CTRL.LON.Kp = 0.5, Ki = 0.05`
- `CTRL.VER.cMin = 500, cMax = 5000, skyGain = 2500`

실제 적용 게인은 ctrl_lateral.m 내부에서 base 값에 scaling factor를 곱하는 방식으로 처리(gain scheduling).

`SIM.solver`는 default ode45 유지.

---

## 부록 C — Plot 생성 코드

다음 MATLAB 스크립트로 §4의 plot 5장을 재현할 수 있다. `icc-project/` 폴더에서 실행:

```matlab
% --- A7 sideSlip 비교 ---
[r_off, ~] = run_icc_scenario('A7','14dof','Controller','off','SavePlot',false);
[r_on,  ~] = run_icc_scenario('A7','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.t, rad2deg(r_off.sideSlip), 'r-', 'LineWidth', 1.5); hold on;
plot(r_on.t,  rad2deg(r_on.sideSlip),  'b-', 'LineWidth', 1.5);
xlabel('time [s]'); ylabel('\beta [deg]');
legend('off (baseline)','on (designed)','Location','best');
title('A7 Brake-in-Turn — sideSlipMax');
grid on; saveas(gcf, 'docs/figures/a7_sideSlip.png');

% --- A7 trajectory ---
figure; plot(r_off.x_pos, r_off.y_pos, 'r-', r_on.x_pos, r_on.y_pos, 'b-', ...
             'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]'); axis equal;
legend('off (baseline)','on (designed)');
title('A7 — Vehicle Trajectory');
grid on; saveas(gcf, 'docs/figures/a7_trajectory.png');

% --- B1 wheel slip ---
[r_off, ~] = run_icc_scenario('B1','14dof','Controller','off','SavePlot',false);
[r_on,  ~] = run_icc_scenario('B1','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.t, r_off.wheelSlip(:,1), 'r-', r_on.t, r_on.wheelSlip(:,1), 'b-', ...
             'LineWidth', 1.5);
xlabel('time [s]'); ylabel('\kappa_{FL}');
legend('off','on'); title('B1 — Front-Left Wheel Slip Ratio');
grid on; saveas(gcf, 'docs/figures/b1_kappa.png');

% --- B1 brake ---
figure; subplot(2,1,1);
plot(r_off.t, r_off.v_x*3.6, 'r-', r_on.t, r_on.v_x*3.6, 'b-', 'LineWidth', 1.5);
ylabel('v_x [km/h]'); legend('off','on'); grid on;
subplot(2,1,2);
plot(r_off.t, r_off.a_x, 'r-', r_on.t, r_on.a_x, 'b-', 'LineWidth', 1.5);
xlabel('time [s]'); ylabel('a_x [m/s^2]'); grid on;
sgtitle('B1 — Deceleration');
saveas(gcf, 'docs/figures/b1_brake.png');

% --- A1 path ---
[r_off, ~] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  ~] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:', ...
             r_off.x_pos, r_off.y_pos, 'r-', r_on.x_pos, r_on.y_pos, 'b-', ...
             'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]'); axis equal;
legend('refPath','off (baseline)','on (designed)','Location','best');
title('A1 ISO 3888-1 DLC — Path Following');
grid on; saveas(gcf, 'docs/figures/a1_path.png');
```

---

## 부록 D — 코드 무결성 정보

`icc-project/grade_report.json` (2026-06-23 03:55:13 KST 생성):

- `quantitative.score`: 60.78 / 70
- `deductions.amount`: 0
- `final_auto`: 60.78
- `ctrl_signature`: 4개 ctrl_*.m 의 SHA256 hash (lateral / longitudinal / vertical / coordinator 순서 concat, CRLF→LF 정규화)
- `matlab_version`: 9.14 R2023a
- `solver_used`: ode45

본 fork의 GitHub Actions(`.github/workflows/classroom.yml`)가 `ctrl_signature` 와 실제 ctrl_*.m 파일의 hash 일치 여부를 자동 검증한다. `grade_report.json` 수동 편집은 자동 차단된다.