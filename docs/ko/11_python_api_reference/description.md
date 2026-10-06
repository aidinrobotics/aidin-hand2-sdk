[← Python API Reference](../11_python_api_reference.md)

# Hand description

| Symbol | Kind | Description |
|---|---|---|
| [상수](#constants) | `int` | actuator·joint·tactile 개수 |
| [`HandSide`](#enum-handside) | `enum.Enum` | 왼손 / 오른손 |
| [`Finger`](#enum-finger) | `enum.IntEnum` | tactile finger index |
| [배열 배치](#array-layout) | | 배열의 길이·단위·index |

---

## Constants

배열 크기는 숫자 대신 아래 상수를 쓰십시오.

```python
ACTUATOR_COUNT = 16              # actuator state·command·gain
JOINT_COUNT = 21                 # passive 포함 joint state
ACTIVE_JOINT_COUNT = 16          # joint command
PASSIVE_JOINT_COUNT = 5          # finger마다 1개인 passive q4
FINGER_COUNT = 5                 # thumb·index·middle·ring·baby
TACTILE_TAXELS_PER_FINGER = 17
PALM_TACTILE_COUNT = 58          # palm1(20+20) + palm2(18)
PALM1_UPPER_COUNT = 20
PALM1_LOWER_COUNT = 20
PALM2_COUNT = 18
```

---

## enum `HandSide`

```python
class HandSide(enum.Enum):
    LEFT = 0
    RIGHT = 1
```

---

## enum `Finger`

```python
class Finger(enum.IntEnum):
    THUMB = 0
    INDEX = 1
    MIDDLE = 2
    RING = 3
    BABY = 4
```

정수로 쓸 수 있으므로 `state.tactile.fingers[Finger.INDEX]`처럼 배열 index로 바로 넘깁니다.

---

## Array layout

길이와 단위가 맞는지 확인하고 쓰십시오.

| Data | dtype · shape | Unit |
|---|---|---|
| Actuator position | `int32` `(16,)` | encoder count |
| Actuator velocity | `int32` `(16,)` | rpm |
| Actuator current | `int16` `(16,)` | mA |
| Actuator effort command · limit | `float64` `(16,)` | 정격 전류의 0.1% |
| Active joint command | `float64` `(16,)` | rad |
| Joint position | `float64` `(21,)` | rad |

index 공간은 세 가지이며, 배치는 C++ SDK와 같습니다. index별 finger 표는
[C++ API reference](../08_cpp_api_reference/types_description.md#배열-규약)에 있습니다.

- **actuator index**: 길이 16. actuator의 position·velocity·current, effort command·limit,
  impedance gain. thumb이 0~3이고 index·middle·ring·baby finger가 3개씩 이어집니다
- **active joint index**: 길이 16. joint command. actuator index와 finger별 묶음이 같지만, 같은
  index의 joint와 actuator가 같은 회전축을 뜻하지는 않습니다
- **joint index**: 길이 21. [`JointState`](state.md#jointstate)의 position. finger마다 마지막
  자리에 passive joint가 하나씩 들어갑니다

joint index 21개에서 active joint 16개를 고르려면 passive joint 자리를 빼면 됩니다.

```python
ACTIVE_IN_JOINT = [0, 1, 2, 3,  5, 6, 7,  9, 10, 11,  13, 14, 15,  17, 18, 19]

active = state.joints.position_rad[ACTIVE_IN_JOINT]   # float64, shape (16,)
```
