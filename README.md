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
| Subscribe | `/actuators/valve_voltage`（パラメータ `subscribe_topic` で変更可） | `Float32MultiArray` | バルブへの指令電圧 [V]（0–10V、5V中立、8ch） |

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

---

### cylinder_exp パッケージ

#### `pam_const_pressure_controller` ノード

PI制御により PAM 圧力を一定値（デフォルト 100 kPa）に保つ。
バルブ指令は `mixer_node` 経由で AO ボードに送られる。

| 種別 | トピック名 | 型 | 説明 |
|------|-----------|-----|------|
| Subscribe | `/sensors/pam_pressure`（パラメータ `pressure_topic` で変更可） | `Float32` | PAM圧力 [kPa] |
| Publish | `/actuators/pam_valve`（パラメータ `valve_topic` で変更可） | `Float32MultiArray` | `[ch, volt]` 形式のバルブ指令（デフォルト ch=3） |
| Publish | `/debug/pam_target_pressure_kPa` | `Float32` | 目標圧力 [kPa] |
| Publish | `/debug/pam_pressure_error_kPa` | `Float32` | 圧力偏差 [kPa] |
| Publish | `/debug/pam_valve_output_V` | `Float32` | PI出力（中立からの差分電圧）[V] |

#### `mixer_node` (`valve_mixer`) ノード

複数の制御ノードからのバルブ指令（`[ch, volt, ch, volt, ...]` 形式）を
統合し、8ch の `Float32MultiArray` として一定周期で出力する。

| 種別 | トピック名 | 型 | 説明 |
|------|-----------|-----|------|
| Subscribe | `/actuators/cylinder_valves`（パラメータ `input_topics` に含まれる） | `Float32MultiArray` | `[ch, volt, ...]` シリンダ制御側のバルブ指令 |
| Subscribe | `/actuators/pam_valve`（パラメータ `input_topics` に含まれる） | `Float32MultiArray` | `[ch, volt, ...]` PAM制御側のバルブ指令 |
| Publish | `/actuators/valve_voltage` | `Float32MultiArray` | 統合後の8chバルブ電圧 [V]（未指定chは中立5V） |

---

## バルブ指令のフロー

```
[pam_const_pressure_controller] ──► /actuators/pam_valve      ─┐
                                                                ├─► [mixer_node] ──► /actuators/valve_voltage ──► [ao1608llpe]
[他の制御ノード]                 ──► /actuators/cylinder_valves ─┘
```

各制御ノードは `[ch, volt]` 形式で自分が担当するchのみ指令を出し、
`mixer_node` がそれらを8ch統合してAOボードへ送る。
