# Logistics Center

긴급 주문의 피킹 마감 준수율에 크레인 포크 수, AGV 트레일러 수,
배차 정책이 미치는 영향을 비교하는 SOCS 예제입니다.

## 실행

SOCS가 설치된 Python 환경에서 저장소 루트 기준으로 실행합니다.
소스 체크아웃에서는 먼저 PowerShell에서 `$env:PYTHONPATH = 'src'`를 설정할 수 있습니다.

```powershell
python -m examples.LogisticsCenter --view --flow out
python -m examples.LogisticsCenter --sweep --flow all
```

3D 화면에는 Panda3D가 필요합니다.

| 조작 | 동작 |
|---|---|
| **Start** 또는 Space | 시작·재개 |
| P | 일시정지 |
| **Reset** 또는 R | 초기 상태로 되돌리고 정지 |
| U / D | 재생 속도 2배 / 절반. 범위 0.25–16배 |
| Esc | 종료 |

화면 상단에 시뮬레이션 시간, 배속, 흐름, 설비 작업, 포트 점유,
완료 주문 수를 표시합니다. 랙 양 끝의 L1/L2와 R1/R2는 양방향 포트입니다.

## 흐름

| 옵션 | 출고 | 기존 Bin 회수 | 신규 입고 |
|---|---|---|---|
| `--flow out` | 예 | | |
| `--flow in` | | | 예 |
| `--flow out_return` | 예 | 예 | |
| `--flow all` (기본값) | 예 | 예 | 예 |

`out`에서는 피킹이 끝난 Bin이 시스템에서 빠집니다. `in`에서는 초기 랙
재고에 신규 Bin을 추가합니다.

## 모델

최상위 DEVSPetriNet이 주문과 재고를 배정하고 크레인, 견인 AGV(기본 2대),
작업대는 DEVSph로 작업을 실행합니다. 랙은 15열 × 6단, Bin당 최대
12박스입니다. 네 포트는 각 1 Bin, 작업대의 유입·유출은 각 2 Bin을 수용합니다.
납기는 마지막 SKU의 피킹 완료를 기준으로 평가합니다.

크레인은 기존 AS/RS 예제의 축별 가감속 프로파일을 공유하고 포크를
전진·상승·복귀시킵니다. AGV는 교차로가 없는 일방향 순환로와 별도 대기
베이를 사용합니다. 코너 라운딩, 정차 전 감속과 출발 시 가속을 소요시간에 반영합니다.
견인차와 카트는 기존 AGVSystem의 3D 메시를 사용합니다.

순환로는 0.75 m 점유 블록으로 나누고, 0.25초 간격으로 견인차와 카트의 공간을 예약합니다. 겹치지 않는 블록에서는 차량이 동시에 주행합니다. 카트는 경로상 거리 간격으로
표시합니다. 연결부 관절과 횡방향 궤적은 단순화했습니다. 두 포트와 유입
버퍼 제약으로 한 운행에서 최대 2 Bin을 처리합니다. 속도와 주문 마감은 실험 입력값입니다.

## 포크 정렬과 측면 인계

포크 중심은 크레인 중심에서 각각 -0.5 m, +0.5 m입니다.
두 출고 위치가 필요하면 한쪽의 1번과 2번 포트를 우선 예약합니다.
AGV 순환로는 포트 중심선에서 2.2 m 떨어져 있고, 각 카트가 포트와
나란히 정차한 뒤 Bin이 측면으로 전달됩니다. 견인차는 기존 메시를
사용하고 카트는 모서리를 깎은 다각형 차체와 원형 바퀴로 표시합니다.

## SKU inventory

The HUD shows box counts by SKU as AS/RS available / order allocated.
Rack availability subtracts reservations immediately. Allocations remain on
open orders even after individual items are picked and are released when the
last item is picked, the shipment boundary in this model. Empty Bins leave
at the picking station and release their rack cells. Nonempty Bins return
when the selected flow enables returns.

In outbound-enabled modes, after the initial orders the generator submits one
order for A, B and C every 15 simulated seconds from t=60 until the run limit.
Use `--flow out_return` to observe returns and eventual depletion without inbound.

## Pull replenishment

In `all` and `in`, the warehouse orders one 12-box Bin per SKU when its
unreserved warehouse stock reaches or falls below `--reorder-point` (default 12).
The threshold includes unreserved stock in Bins circulating inside the warehouse.
The supplier sends a Bin to external receiving after `--lead-time` simulated
seconds (default 60). Receiving, AGV transport and putaway follow. An SKU gets
no second pull order while its replenishment is pending or awaiting putaway.
There are no scheduled supplier pushes. `out` and `out_return` disable replenishment.
For an inbound-only demo with 36 initial boxes per SKU, use a threshold of 36.

```powershell
python -m examples.LogisticsCenter --view --flow all --reorder-point 18 --lead-time 90
```

## Concurrent AGVs

Use `--agvs 1` through `--agvs 4` with any flow, including the viewer and sweep.
A vehicle waits in its own bay until it can reserve the short blocks along its
trip without overlapping other reservations. This is a discrete occupancy
model using trajectory samples, not a continuous collision detector.

```powershell
python -m examples.LogisticsCenter --view --flow all --agvs 4 --trailers 2
```
