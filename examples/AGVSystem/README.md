# AGVSystem

AS/RS와 분리된 단방향 AGV 예제. Blender 장면과 SOCS 모델은 `layout.py`의 동일한 좌표, 거리, 속도를 사용한다. 단위는 m와 s다.

## 치수

| 항목 | 값 |
|---|---:|
| 바닥 | 40 × 26 m |
| 경로 중심선 외곽 | 32 × 18 m |
| 차로 폭 | 2.4 m |
| 직선 구간 | 8 m × 6, 10 m × 2 |
| 원호 구간 | 반경 4 m, 90° × 4 |
| 전체 중심선 길이 | 93.13 m |
| 제어점 | 12개, 충전 정지점 2개 포함 |
| 충전점 좌표 | CH01 (−4, −9), CH02 (4, 9) |
| 견인형 + 카트 2대 | 4.40 × 0.95 m, 1.10 m/s |
| 단위화물형 | 1.60 × 1.00 m, 1.30 m/s |
| 지게차형, 포크 포함 | 2.65 × 1.15 m, 1.00 m/s |

## 모델과 애니메이션

- `ControlPoint`는 도착을 보고하고, 정차 시간이 지나면 출발 가능을 알린다. 제어기의 허가가 오면 연결된 세그먼트에 출발 이벤트를 보낸다.
- `PathSegment`는 차량의 앞부분 진입, 뒷부분의 출발점 통과, 다음 점 도착을 시간 이벤트로 처리한다.
- `Controller`는 SOCS `AGraph`의 정점(제어점)과 방향 간선(세그먼트)을 조회한다. 다음 세그먼트와 도착점을 함께 예약하고, 진입 보고로 점유를 기록한다. 차량 후미가 출발점을 통과하면 출발점과 이전 세그먼트를 해제한다.
- `make_model()`은 이 모델들을 CoupledDEVS로 연결한다. 차량은 메시지에 실리는 ID와 치수·속도 프로필로 표현한다.
- Blender에서 타임라인을 재생하면 SOCS의 90초 이벤트 기록을 바탕으로 차량이 움직인다. 첫 프레임은 초기 배치다.

예제의 모든 차량은 순환 주행하며 두 충전점에서 각각 7초간 정차한다. 충전기는 본선 옆에 놓이고 차량은 본선에서 멈춘다. 충전량, 화물 이송, 가감속, 추종 제어 및 Traffic Zone은 생략했다. 각 세그먼트는 배타적으로 예약된다. 카트의 모습과 점유는 중심선 거리를 이용한 근사이며 실제 조향·견인 기구나 안전 간격 검증용 치수가 아니다.

저장소 루트에서 DEVS 기록을 확인하려면 다음을 실행한다.

```powershell
$env:PYTHONPATH = 'src'
python -m examples.AGVSystem.agv_devs
```

Blender를 PATH에서 실행할 수 있다면 다음 명령으로 장면과 프리뷰를 재생성할 수 있다.

```powershell
blender -b --python-exit-code 1 --python examples/AGVSystem/build_blender.py
```

## Panda3D 실시간 실행

저장소 루트의 PowerShell에서 실행한다. 이 명령은 Blender에 포함된 Python 3.13과 저장소의 `.venv`에 설치된 Panda3D를 사용한다.

```powershell
$env:PYTHONPATH = 'src;.venv/Lib/site-packages'
& 'C:\Program Files\Blender Foundation\Blender 5.2\5.2\python\bin\python.exe' -m examples.AGVSystem.run_panda
```

Python 3.13 환경에 SOCS와 Panda3D가 이미 설치되어 있다면 `python -m examples.AGVSystem.run_panda`로 실행할 수 있다. 인수 `--speed 2`를 추가하면 두 배속으로 실행한다. Space는 일시 정지/재개, R은 초기화, 방향키는 시점 회전, 마우스 휠은 확대/축소다.

Panda3D가 화면 프레임마다 현재 시뮬레이션 시간까지 SOCS 이벤트를 처리한다. DEVS는 출발과 도착 시점을 결정하고, Panda3D는 그 사이 차량과 카트의 위치를 경로 위에서 보간한다. Blender 모델의 형상을 수정하면 아래 명령으로 뷰어용 형상을 다시 내보낸다.

```powershell
& 'C:\Program Files\Blender Foundation\Blender 5.2\blender.exe' -b .\examples\AGVSystem\AGVSystem.blend --python-exit-code 1 --python .\examples\AGVSystem\export_panda.py
```

기본 화면은 왼쪽 3D 공장뷰와 오른쪽 2D 관제뷰다. **Tab**을 누르면 3D 전체 화면으로 전환되고, 다시 누르면 두 화면이 동시에 표시된다. 관제뷰는 동일한 시뮬레이션의 예약(노란 점선), 점유(파란 실선), 대기(WAIT), 충전(CHG)을 표시한다.

[포트 결합과 상태천이도](state_transitions.md)

## 치수 변경

[dimensions.json](dimensions.json)에서 경로 중심선의 폭·깊이, 곡선 반경, 차로 폭, 바닥 여백, 직선 위 정지점 위치 비율, 충전기 치수 및 정차 시간, 차량별 길이·폭·높이·속도·출발점을 바꿀 수 있다. 거리 단위는 m, 시간 단위는 s다. 차량의 길이·폭·높이는 제공된 형상의 비율을 유지하며 각 축을 스케일한다. 견인형 길이에는 카트 두 대가 포함된다.

DEVS는 다음 실행부터 설정값을 읽는다. 3D 형상과 Blender 애니메이션도 갱신하려면 아래 명령을 실행한다. 이 명령은 Blender 장면, 프리뷰, Panda3D 형상을 함께 갱신한다.

```powershell
& 'C:\Program Files\Blender Foundation\Blender 5.2\blender.exe' -b --python-exit-code 1 --python .\examples\AGVSystem\build_blender.py
```

Panda3D는 [Run_AGV.cmd](Run_AGV.cmd)를 더블클릭해 실행할 수 있다. Windows 탐색기에서 이 파일의 **보내기 → 바탕 화면에 바로 가기 만들기**를 선택하면 바탕화면에서도 실행할 수 있다.
