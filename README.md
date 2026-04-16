# cart_detect_sim

ROS 2 simulation mock for `cart_detect` services.

Provides the `detect` and `cartdetect/enable_detection` services that the
Commander's CartScanRoutine expects. Cart positions are configurable via
YAML parameters and can be added/moved/removed at runtime via services.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `detect` | `cart_detect::srv::GetCarts` | Returns carts within detection range of the robot |
| `cartdetect/enable_detection` | `std_srvs::srv::SetBool` | Enable/disable detection (OFF returns empty) |
| `sim/set_cart` | `cart_detect_sim::srv::SetCart` | Add or move a cart |
| `sim/remove_cart` | `cart_detect_sim::srv::RemoveCart` | Remove a cart |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_detection_distance` | double | 5.0 | Max distance (m) from robot to detect a cart |
| `map_frame_id` | string | "map" | TF frame for cart world positions |
| `initial_carts` | string[] | [] | List of cart parameter group names to load |

Each cart group (e.g. `cart_30`) has:

| Parameter | Type | Description |
|-----------|------|-------------|
| `cart_30.id` | int | Cart ID (30-99) |
| `cart_30.type` | int | Cart type |
| `cart_30.x` | double | X position in map frame |
| `cart_30.y` | double | Y position in map frame |
| `cart_30.yaw` | double | Orientation in map frame (rad) |

## Usage

```bash
# Launch with default config
ros2 launch cart_detect_sim cart_detect_sim.launch.py

# Add a cart at runtime
ros2 service call /sim/set_cart cart_detect_sim/srv/SetCart \
  "{id: 31, type: 1, x: 7.0, y: -1.0, yaw: 1.57}"

# Move an existing cart
ros2 service call /sim/set_cart cart_detect_sim/srv/SetCart \
  "{id: 30, type: 1, x: 8.0, y: 2.0, yaw: 0.0}"

# Remove a cart
ros2 service call /sim/remove_cart cart_detect_sim/srv/RemoveCart "{id: 31}"

# Enable detection (Commander does this automatically)
ros2 service call /cartdetect/enable_detection std_srvs/srv/SetBool "{data: true}"

# Test detect service
ros2 service call /detect cart_detect/srv/GetCarts \
  "{target_frame: 'null', source_frame: 'base_footprint'}"
```
