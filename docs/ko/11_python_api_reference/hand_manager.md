[← Python API Reference](../11_python_api_reference.md)

# HandManager

| Symbol | Kind | Description |
|---|---|---|
| [`HandManager`](#handmanager) | class | hand 자원을 만들고 소유. `with` 블록 지원 |

---

## `HandManager`

manager 자신은 CAN에 연결하지 않습니다. 연결과 제어는 [`create()`](#handmanagercreate)가 돌려준
[`Hand`](hand.md#hand)로 합니다. [`destroy()`](#handmanagerdestroy)나 `with` 블록의 끝에서 정리된
hand를 가리키던 `Hand`는 무효가 됩니다.

| Member | Description |
|---|---|
| [`HandManager()`](#handmanager__init__) | 빈 manager |
| [`create()`](#handmanagercreate) | config를 검증하고 hand를 만듦 |
| [`destroy()`](#handmanagerdestroy) | hand 하나를 정리 |
| [`destroy_all()`](#handmanagerdestroy_all) | 소유한 hand를 전부 정리 |
| [`with` 블록](#context-manager) | 블록의 끝에서 `destroy_all()` |

정리는 actuator quick stop을 확인하고 연결을 끊은 뒤 자원을 반납하는 것을 말합니다. `with` 블록이나
`destroy()`·`destroy_all()`을 쓰지 않으면 manager와 그 manager가 만든 `Hand`가 모두 해제될 때
정리되고, 남아 있으면 프로그램이 끝날 때 정리됩니다.

---

### `HandManager.__init__()`

```python
def __init__(self) -> None: ...
```

hand가 없는 manager를 만듭니다.

---

### `HandManager.create()`

```python
def create(self, config: HandConfig) -> Hand: ...
```

config를 검증하고 hand 하나를 만듭니다. 이 시점에는 CAN에 연결하지 않으며, 연결은 돌려받은
`Hand`의 [`connect()`](hand.md#handconnect)로 합니다.

**Parameters**     ｜ `config` — 만들 hand의 설정 [`HandConfig`](config.md#handconfig)<br>
**Returns**        ｜ 새로 만든 hand를 가리키는 [`Hand`](hand.md#hand)<br>
**Raises**         ｜ `Error`(`INVALID_ARGUMENT`) — `config` 값이 유효하지 않을 때<br>
**Notes**          ｜ 돌려받은 `Hand`는 이 manager를 참조하므로, `Hand`가 살아 있는 동안 manager도
유지됩니다

---

### `HandManager.destroy()`

```python
def destroy(self, hand: Hand) -> None: ...
```

`hand`가 가리키는 대상을 정리합니다. 연결 중이면 actuator quick stop을 확인한 뒤 연결을 끊습니다.
확인하지 못해도 예외 없이 반납하고, 확인에 실패한 시점에 critical log가 남습니다.

**Parameters**     ｜ `hand` — 정리할 대상<br>
**Postconditions** ｜ 같은 대상을 가리키던 `Hand`가 모두 무효가 됨<br>
**Notes**          ｜ 이미 정리된 `hand`를 넘기면 아무것도 하지 않습니다

---

### `HandManager.destroy_all()`

```python
def destroy_all(self) -> None: ...
```

소유한 hand를 전부 정리합니다.

**Postconditions** ｜ manager가 비고, 기존 `Hand`가 모두 무효가 됨

---

### Context manager

```python
with HandManager() as manager:
    ...
```

`with`는 manager 자신을 돌려주고, 블록을 벗어날 때 [`destroy_all()`](#handmanagerdestroy_all)을
호출합니다. 예외나 `KeyboardInterrupt`로 벗어나도 호출하며, 그 예외는 그대로 전파됩니다.
