# koni_ws

## トピック一覧

### control_box パッケージ

#### `ai1616llpe` ノード

| 種別 | トピック名 | 型 | 説明 |
|------|-----------|-----|------|
| Publish | `ai1616llpe/voltage` | `Float32MultiArray` | AI-1616L-LPE の16ch電圧値 [V] |

#### `ao1608llpe` ノード

| 種別 | トピック名 | 型 | 説明 |
|------|-----------|-----|------|
| Subscribe | `/actuators/valve_voltage`（パラメータ `subscribe_topic` で変更可） | `Float32MultiArray` | バルブへの指令電圧 [V]（0–10V、5V中立） |

#### `cnt3204mtlpe` ノード

| 種別 | トピック名 | 型 | 説明 |
|------|-----------|-----|------|
| Publish | `/cnt3204mtlpe` | `Float32MultiArray` | エンコーダのカウント値（初期値からの差分） |

---

### py_signal_processing パッケージ

#### `analog_voltage_interpreter_cyl` ノード

| 種別 | トピック名 | 型 | 説明 |
|------|-----------|-----|------|
| Subscribe | `/ai1616llpe/voltage`（パラメータ `ai_topic` で変更可） | `Float32MultiArray` | AIボードからの電圧入力 |
| Subscribe | `/cnt3204mtlpe`（パラメータ `cnt_topic` で変更可） | `Float32MultiArray` | エンコーダカウント入力 |
| Publish | `/sensors/cylinder_position` | `Float32` | シリンダ位置 [m] |
| Publish | `/sensors/head_pressure` | `Float32` | ヘッド側圧力 [kPa] |
| Publish | `/sensors/rod_pressure` | `Float32` | ロッド側圧力 [kPa] |
| Publish | `/sensors/loadcell_force` | `Float32` | ロードセル力 [N] |
| Publish | `/sensors/pam_pressure` | `Float32` | PAM圧力 [kPa]（AI index=5） |
